import torch
import torchvision.transforms as transforms
from torchvision.models import midas
import matplotlib.pyplot as plt
import cv2
# Load pre-trained MiDaS model
model = midas(pretrained=True)
model.eval()

# Load and preprocess input image
left_img = cv2.imread("/home/erik/catkin_ws/src/VFH/VFH_ROS/DepthMaps/left_DM.png")
preprocess = transforms.Compose([
    transforms.Resize((384, 384)),
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
])
input_tensor = preprocess(left_img)
input_batch = input_tensor.unsqueeze(0)

# Perform depth inference
with torch.no_grad():
    prediction = model(input_batch)

# Display depth map
depth_map = prediction.squeeze().cpu().numpy()
plt.imshow(depth_map, cmap='inferno')
plt.colorbar()
plt.show()
