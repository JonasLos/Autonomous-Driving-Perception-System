from enum import Enum

import cv2
import numpy as np
import scipy.special
import torch
import torchvision.transforms as transforms
from PIL import Image
from UltraFast.model import parsingNet

lane_colors = [
    (0, 0, 255),
    (0, 255, 0),
    (255, 0, 0),
    (0, 255, 255),
]  # Red, Green, Blue, Yellow

tusimple_row_anchor = [
    64,
    68,
    72,
    76,
    80,
    84,
    88,
    92,
    96,
    100,
    104,
    108,
    112,
    116,
    120,
    124,
    128,
    132,
    136,
    140,
    144,
    148,
    152,
    156,
    160,
    164,
    168,
    172,
    176,
    180,
    184,
    188,
    192,
    196,
    200,
    204,
    208,
    212,
    216,
    220,
    224,
    228,
    232,
    236,
    240,
    244,
    248,
    252,
    256,
    260,
    264,
    268,
    272,
    276,
    280,
    284,
]
culane_row_anchor = [
    121,
    131,
    141,
    150,
    160,
    170,
    180,
    189,
    199,
    209,
    219,
    228,
    238,
    248,
    258,
    267,
    277,
    287,
]


class ModelType(Enum):
    TUSIMPLE = 0
    CULANE = 1


class ModelConfig:
    def __init__(self, model_type):
        if model_type == ModelType.TUSIMPLE:
            self.init_tusimple_config()
        else:
            self.init_culane_config()

    def init_tusimple_config(self):
        self.img_w = 1280
        self.img_h = 720
        self.row_anchor = tusimple_row_anchor
        self.griding_num = 100
        self.cls_num_per_lane = 56

    def init_culane_config(self):
        self.img_w = 1640
        self.img_h = 590
        self.row_anchor = culane_row_anchor
        self.griding_num = 200
        self.cls_num_per_lane = 18


class UltrafastLaneDetector:
    def __init__(self, model_path, model_type=ModelType.TUSIMPLE, use_gpu=False):
        self.use_gpu = use_gpu

        # Load model configuration based on the model type
        self.cfg = ModelConfig(model_type)

        # Initialize model
        self.model = self.initialize_model(model_path, self.cfg, use_gpu)

        # Initialize image transformation
        self.img_transform = self.initialize_image_transform()

    @staticmethod
    def initialize_model(model_path, cfg, use_gpu):
        # Load the model architecture
        net = parsingNet(
            pretrained=False,
            backbone="18",
            cls_dim=(cfg.griding_num + 1, cfg.cls_num_per_lane, 4),
            use_aux=False,
        )  # we dont need auxiliary segmentation in testing

        # Load the weights from the downloaded model
        if use_gpu:
            if False:  # torch.backends.mps.is_built():
                net = net.to("mps")
                state_dict = torch.load(model_path, map_location="mps")[
                    "model"
                ]  # Apple GPU
            else:
                net = net.cuda()
                state_dict = torch.load(model_path, map_location="cuda")[
                    "model"
                ]  # CUDA
        else:
            state_dict = torch.load(model_path, map_location="cpu")["model"]  # CPU

        compatible_state_dict = {}
        for k, v in state_dict.items():
            if "module." in k:
                compatible_state_dict[k[7:]] = v
            else:
                compatible_state_dict[k] = v

        # Load the weights into the model
        net.load_state_dict(compatible_state_dict, strict=False)
        net.eval()

        ############ Model Size #################
        # Print the number of parameters
        num_params = sum(p.numel() for p in net.parameters())
        print(f"\nNumber of parameters: {num_params}")

        # Calculate the memory required by the model
        param_size = 0
        for param in net.parameters():
            param_size += param.nelement() * param.element_size()
        buffer_size = 0
        for buffer in net.buffers():
            buffer_size += buffer.nelement() * buffer.element_size()
        size_all_mb = (param_size + buffer_size) / 1024**2
        print(f"Model size: {size_all_mb:.2f} MB")

        return net

    @staticmethod
    def initialize_image_transform():
        # Create transfom operation to resize and normalize the input images
        img_transforms = transforms.Compose(
            [
                transforms.Resize((288, 800)),
                transforms.ToTensor(),
                transforms.Normalize((0.485, 0.456, 0.406), (0.229, 0.224, 0.225)),
            ]
        )

        return img_transforms

    def detect_lanes(self, image, draw_points=True, point_coords=[(0, 0)]):
        input_tensor = self.prepare_input(image)

        # Perform inference on the image
        output = self.inference(input_tensor)

        # Process output data
        self.lanes_points, self.lanes_detected = self.process_output(output, self.cfg)

        # Draw depth image
        visualization_img, lanes_points, lanes_detected = self.draw_lanes(
            image,
            self.lanes_points,
            self.lanes_detected,
            self.cfg,
            draw_points,
            point_coords,
        )

        return visualization_img, self.lanes_points, self.lanes_detected

    def prepare_input(self, img):
        # Transform the image for inference
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img_pil = Image.fromarray(img)
        input_img = self.img_transform(img_pil)
        input_tensor = input_img[None, ...]

        if self.use_gpu:
            if True:  # not torch.backends.mps.is_built():
                input_tensor = input_tensor.cuda()

        return input_tensor

    def inference(self, input_tensor):
        with torch.no_grad():
            output = self.model(input_tensor)

        return output

    @staticmethod
    def process_output(output, cfg):
        # Parse the output of the model
        processed_output = output[0].data.cpu().numpy()
        processed_output = processed_output[:, ::-1, :]
        prob = scipy.special.softmax(processed_output[:-1, :, :], axis=0)
        idx = np.arange(cfg.griding_num) + 1
        idx = idx.reshape(-1, 1, 1)
        loc = np.sum(prob * idx, axis=0)
        processed_output = np.argmax(processed_output, axis=0)
        loc[processed_output == cfg.griding_num] = 0
        processed_output = loc

        col_sample = np.linspace(0, 800 - 1, cfg.griding_num)
        col_sample_w = col_sample[1] - col_sample[0]

        lanes_points = []
        lanes_detected = []

        max_lanes = processed_output.shape[1]
        for lane_num in range(max_lanes):
            lane_points = []
            # Check if there are any points detected in the lane
            if np.sum(processed_output[:, lane_num] != 0) > 2:
                lanes_detected.append(True)

                # Process each of the points for each lane
                for point_num in range(processed_output.shape[0]):
                    if processed_output[point_num, lane_num] > 0:
                        lane_point = [
                            int(
                                processed_output[point_num, lane_num]
                                * col_sample_w
                                * cfg.img_w
                                / 800
                            )
                            - 1,
                            int(
                                cfg.img_h
                                * (
                                    cfg.row_anchor[cfg.cls_num_per_lane - 1 - point_num]
                                    / 288
                                )
                            )
                            - 1,
                        ]
                        lane_points.append(lane_point)
            else:
                lanes_detected.append(False)

            lanes_points.append(lane_points)

        ##################### Modifications ############################
        # Filter out empty lane points and their corresponding detected status
        filtered_lanes_points = [lp for lp in lanes_points if len(lp) > 0]
        filtered_lanes_detected = [
            lanes_detected[i]
            for i in range(len(lanes_points))
            if len(lanes_points[i]) > 0
        ]

        # Pad the lists to ensure they have the same length
        max_length = max(len(lp) for lp in filtered_lanes_points)
        padded_lanes_points = [
            lp + [[0, 0]] * (max_length - len(lp)) for lp in filtered_lanes_points
        ]

        return np.array(padded_lanes_points), np.array(filtered_lanes_detected)

    @staticmethod
    def draw_lanes(
        input_img,
        lanes_points,
        lanes_detected,
        cfg,
        draw_points=True,
        point_coords=[(0, 0)],
    ):
        # Write the detected line points in the image
        visualization_img = cv2.resize(
            input_img, (cfg.img_w, cfg.img_h), interpolation=cv2.INTER_AREA
        )

        # Define colors for filling polygons
        fill_colors = [(255, 191, 0), (0, 255, 0), (0, 0, 255), (255, 0, 255)]

        # Draw a mask for each detected lane
        for i in range(len(lanes_detected) - 1):
            if lanes_detected[i] and lanes_detected[i + 1]:
                lane_segment_img = visualization_img.copy()

                # Filter out [0, 0] points
                lane1_points = [
                    point for point in lanes_points[i] if point[0] != 0 or point[1] != 0
                ]
                lane2_points = [
                    point
                    for point in lanes_points[i + 1]
                    if point[0] != 0 or point[1] != 0
                ]

                if lane1_points and lane2_points:  # Ensure there are points to draw
                    cv2.fillPoly(
                        lane_segment_img,
                        pts=[np.vstack((lane1_points, np.flipud(lane2_points)))],
                        color=fill_colors[i % len(fill_colors)],
                    )
                    visualization_img = cv2.addWeighted(
                        visualization_img, 0.7, lane_segment_img, 0.3, 0
                    )

        if draw_points:
            for lane_num, lane_points in enumerate(lanes_points):
                for lane_point in lane_points:
                    if (
                        lane_point[0] != 0 or lane_point[1] != 0
                    ):  # Filter out points that are [0, 0]
                        cv2.circle(
                            visualization_img,
                            (lane_point[0], lane_point[1]),
                            3,
                            lane_colors[lane_num % len(lane_colors)],
                            -1,
                        )

        # Draw the multiple black points
        for point_coord in point_coords:
            cv2.circle(visualization_img, point_coord, 7, (0, 0, 0), -1)

        return visualization_img, lanes_points, lanes_detected
