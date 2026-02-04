import cv2
import torch
from libs.datasets.metrics.culane_metric import interp
from libs.datasets.pipelines import Compose


def inference_one_image(model, img):
    """Inference on an image array with the detector.
    Args:
        model (nn.Module): The loaded detector.
        img (np.ndarray): BGR image array of shape (H, W, C).
    Returns:
        img (np.ndarray): The original input image.
        preds (List[np.ndarray]): Detected lanes as lists of (x, y) points.
    """
    # Keep a copy of the original
    ori_img = img.copy()
    ori_shape = ori_img.shape  # (H, W, C)

    # Build the data dict expected by the pipeline
    data = dict(
        filename=None,
        sub_img_name=None,
        img=ori_img,
        gt_points=[],
        id_classes=[],
        id_instances=[],
        img_shape=ori_shape,
        ori_shape=ori_shape,
    )

    # Prepare the test pipeline
    cfg = model.cfg
    model.bbox_head.test_cfg.as_lanes = False
    test_pipeline = Compose(cfg.test_dataloader.dataset.pipeline)

    # Apply transforms
    data = test_pipeline(data)
    data_batch = dict(
        inputs=[data["inputs"]],
        data_samples=[data["data_samples"]],
    )

    # Forward pass
    with torch.no_grad():
        results = model.test_step(data_batch)

    # Extract and post-process lane predictions
    lanes = results[0]["lanes"]
    preds = get_prediction(lanes, ori_shape[0], ori_shape[1])

    return ori_img, preds


def get_prediction(lanes, ori_h, ori_w):
    preds = []
    for lane in lanes:
        lane = lane.cpu().numpy()
        xs = lane[:, 0]
        ys = lane[:, 1]
        valid_mask = (xs >= 0) & (xs < 1)
        xs = xs * ori_w
        lane_xs = xs[valid_mask]
        lane_ys = ys[valid_mask] * ori_h
        lane_xs, lane_ys = lane_xs[::-1], lane_ys[::-1]
        pred = [(x, y) for x, y in zip(lane_xs, lane_ys)]
        interp_pred = interp(pred, n=5)
        preds.append(interp_pred)
    return preds
