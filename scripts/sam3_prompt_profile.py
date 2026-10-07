"""Where does SAM3's per-frame time go, and can several prompts be cheaper than N x one?

Run inside the SAM3 image on GPU 1, with frames saved as /prof/frame_*.npy (BGR, any size):

  docker run --rm --gpus all -e CUDA_VISIBLE_DEVICES=1 -e CUDA_DEVICE_ORDER=PCI_BUS_ID \
    -v $PWD:/root/ws -v $HOME/.cache/huggingface:/root/.cache/huggingface -v FRAMES:/prof \
    --entrypoint bash test-sam3:latest -lc \
    'cd /root/ws/src/SAM3_ROS_NODE && .venv/bin/python /root/ws/scripts/sam3_prompt_profile.py'


Runs inside the SAM3 container on real Route2 frames, mirroring segmentation.py: resize to
max 1024, fp16 autocast, Sam3Processor(confidence_threshold=0.3). Every timing is bracketed by
torch.cuda.synchronize() so GPU work is attributed to the stage that queued it.

Switches (environment): COMPILE=1 builds with sam3's own compile; GRAPHS=1 replays the batched
grounding pass as CUDA graphs; WDTYPE=fp32|fp16|bf16 sets the weight dtype (fp16/bf16 fail:
SAM3 mixes fp32 tensors internally); SAVE=tag writes each frame's batched masks to
/prof/masks_<tag>_NN.npz so two runs can be compared mask for mask; N_WARM sets warm-up frames.
"""
import contextlib
import glob
import os
import sys
import time

import cv2
import numpy as np
import torch
from torch.nn.functional import interpolate

import sam3
import sam3.model.sam3_image_processor as sp
from sam3.model import box_ops
from sam3.model.data_misc import FindStage

DEV = "cuda"
TERMS = [t.strip() for t in os.environ.get("TERMS", "paved road,gravel road,car,person").split(",")]
N_WARM = int(os.environ.get("N_WARM", "3"))

_t_build = time.perf_counter()
model = sam3.build_sam3_image_model(compile=bool(int(os.environ.get("COMPILE", "0"))))
print(f"build {time.perf_counter() - _t_build:.1f} s, compile={os.environ.get('COMPILE', '0')}", flush=True)
WDTYPE = {"fp32": torch.float32, "fp16": torch.float16, "bf16": torch.bfloat16}[os.environ.get("WDTYPE", "fp32")]
model.to(device=DEV, dtype=WDTYPE).eval()
proc = sp.Sam3Processor(model, confidence_threshold=0.3)
GROUND = model.forward_grounding
if os.environ.get("GRAPHS") == "1":
    # geometry_encoder._encode_boxes pins a host tensor (untraceable); it runs eager.
    _ge = model.geometry_encoder
    _ge._encode_boxes = torch.compiler.disable(_ge._encode_boxes)
    GROUND = torch.compile(model.forward_grounding, mode="reduce-overhead", dynamic=False)
frames = [np.load(f) for f in sorted(glob.glob("/prof/frame_*.npy"))]


def amp():
    return torch.autocast("cuda", dtype=torch.bfloat16 if WDTYPE == torch.bfloat16 else torch.float16)


def prep(img):
    h, w = img.shape[:2]
    s = 1024 / max(h, w)
    img = cv2.resize(img, (int(w * s), int(h * s)))
    return torch.from_numpy(img.transpose(2, 0, 1)).to(DEV, torch.float32).div(255).to(WDTYPE).contiguous()


def timed(fn):
    torch.cuda.synchronize()
    t = time.perf_counter()
    r = fn()
    torch.cuda.synchronize()
    return r, 1e3 * (time.perf_counter() - t)


def postprocess(outputs, state, k=None):
    """Sam3Processor._forward_grounding's post-processing, optionally for query k of a batch."""
    logits, boxes_, masks = outputs["pred_logits"], outputs["pred_boxes"], outputs["pred_masks"]
    pres = outputs["presence_logit_dec"]
    if k is not None:
        logits, boxes_, masks, pres = logits[k:k + 1], boxes_[k:k + 1], masks[k:k + 1], pres[k:k + 1]
    probs = (logits.sigmoid() * pres.sigmoid().unsqueeze(1)).squeeze(-1)
    keep = probs > proc.confidence_threshold
    m = masks[keep]
    m = interpolate(m.unsqueeze(1), (state["original_height"], state["original_width"]),
                    mode="bilinear", align_corners=False).sigmoid()
    return (m > 0.5)[:, 0], probs[keep]


rows = {k: [] for k in ("set_image", "seq_text", "seq_ground", "seq_total", "cached_total",
                        "batch_total", "batch_core", "post_node")}
agree = []
with torch.inference_mode(), amp():
    # Text encodings never change between frames: compute once.
    cached = {t: model.backbone.forward_text([t], device=DEV) for t in TERMS}
    batch_text = model.backbone.forward_text(TERMS, device=DEV)
    K = len(TERMS)
    fs_k = FindStage(img_ids=torch.zeros(K, device=DEV, dtype=torch.long),
                     text_ids=torch.arange(K, device=DEV, dtype=torch.long),
                     input_boxes=None, input_boxes_mask=None, input_boxes_label=None,
                     input_points=None, input_points_mask=None)
    for i, img in enumerate(frames):
        x = prep(img)
        state, t_img = timed(lambda: proc.set_image(x))

        # 1) today's path: per term, forward_text + grounding + post-processing
        seq_masks, t_text, t_ground = [], 0.0, 0.0
        for t in TERMS:
            txt, dt = timed(lambda: model.backbone.forward_text([t], device=DEV))
            t_text += dt
            state["backbone_out"].update(txt)
            state.setdefault("geometric_prompt", model._get_dummy_prompt())
            st, dt = timed(lambda: proc._forward_grounding(state))
            t_ground += dt
            m = st["masks"].reshape(-1, *st["masks"].shape[-2:])
            seq_masks.append(m.any(dim=0) if m.shape[0] else
                             torch.zeros(m.shape[-2:], dtype=torch.bool, device=DEV))

        # 2) cached text encodings, still one grounding pass per term
        def cached_path():
            for t in TERMS:
                state["backbone_out"].update(cached[t])
                proc._forward_grounding(state)
        _, t_cached = timed(cached_path)

        # 3) one batched grounding pass for all K terms
        def batched_core():
            state["backbone_out"].update(batch_text)
            return GROUND(backbone_out=state["backbone_out"], find_input=fs_k,
                                           geometric_prompt=model._get_dummy_prompt(num_prompts=K),
                                           find_target=None)
        out, t_core = timed(batched_core)

        def batched_post():
            return [postprocess(out, state, k)[0] for k in range(K)]
        bm, t_bpost = timed(batched_post)
        if os.environ.get("SAVE"):
            np.savez_compressed(f"/prof/masks_{os.environ['SAVE']}_{i:02d}.npz",
                                *[(m.any(dim=0) if m.shape[0] else torch.zeros(state["original_height"], state["original_width"], dtype=torch.bool, device=DEV)).cpu().numpy() for m in bm])

        # node-side post-processing on the union masks (mask ids, PNG, overlay at 1024)
        def node_post():
            union = torch.stack([m for m in seq_masks]).any(dim=0).to(torch.uint8).cpu().numpy()
            cv2.imencode(".png", union)
            overlay = np.zeros((*union.shape, 3), np.uint8)
            cv2.addWeighted(cv2.resize(img, union.shape[::-1]), 1.0, overlay, 0.5, 0.0)
        _, t_post = timed(node_post)

        if i < N_WARM:
            print(f"warm-up frame {i}: set_image {t_img:.0f} ms, seq ground {t_ground:.0f} ms, "
                  f"batch core {t_core:.0f} ms", flush=True)
        if i >= N_WARM:
            rows["set_image"].append(t_img)
            rows["seq_text"].append(t_text)
            rows["seq_ground"].append(t_ground)
            rows["seq_total"].append(t_img + t_text + t_ground)
            rows["cached_total"].append(t_img + t_cached)
            rows["batch_core"].append(t_core)
            rows["batch_total"].append(t_img + t_core + t_bpost)
            rows["post_node"].append(t_post)
            for k in range(K):
                a = seq_masks[k]
                b = bm[k].any(dim=0) if bm[k].shape[0] else torch.zeros_like(a)
                u = (a | b).sum().item()
                agree.append(1.0 if u == 0 else (a & b).sum().item() / u)

print(f"GPU {torch.cuda.get_device_name(0)}, {len(frames) - N_WARM} frames, terms={TERMS}")
for k, v in rows.items():
    v = np.array(v)
    print(f"  {k:13s} median {np.median(v):7.1f} ms   p90 {np.percentile(v, 90):7.1f} ms")
print(f"  peak GPU memory {torch.cuda.max_memory_allocated() / 2**30:.2f} GiB")
print(f"  batched vs sequential mask IoU: median {np.median(agree):.3f}, min {np.min(agree):.3f}")
