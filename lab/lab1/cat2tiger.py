"""
Author: Xiang Lei
Created: 2023-09-17 17:28:24
Purpose: 基于特征的图象变形（猫变虎）
"""

import os
import cv2

#####################################################
steps = 100  # 变换步数
show_image = False  # 是否显示中间变形图像
root_path = "/Users/lei/Desktop/CG/lab1"  # 项目根目录
save_image_path = os.path.join(root_path, "result", "image")  # 输出图像目录
save_video_path = os.path.join(root_path, "result", "video")  # 输出视频目录
#####################################################


def image2video(image_folder, fps=30):
    """convert image to video

    Args:
        image_folder (_type_): _description_
        fps (int, optional): _description_. Defaults to 30.
    """
    if not os.path.exists(save_video_path):
        os.makedirs(save_video_path, exist_ok=True)

    image_list = [image for image in os.listdir(
        image_folder) if image.endswith(".png")]
    image_list = sorted(image_list, key=lambda x: int(
        x.split("_")[1].split(".")[0]))

    frame = cv2.imread(os.path.join(image_folder, image_list[0]))
    height, width, layers = frame.shape

    save_path = os.path.join(save_video_path, "cat2tiger_video.mp4")
    video = cv2.VideoWriter(save_path, cv2.VideoWriter_fourcc(
        *'mp4v'), fps, (width, height))

    for image in image_list:
        video.write(cv2.imread(os.path.join(image_folder, image)))

    video.release()


def main():
    """generate cat2tiger images
    """
    if not os.path.exists(save_image_path):
        os.makedirs(save_image_path, exist_ok=True)

    # 读取猫的图像
    cat_path = os.path.join(root_path, 'src/cat.bmp')
    cat = cv2.imread(cat_path)
    # 读取虎的图像
    tiger_path = os.path.join(root_path, 'src/tiger.bmp')
    tiger = cv2.imread(tiger_path)

    for i in range(steps + 1):
        alpha = 1 - i / steps  # 猫的比例
        beta = i / steps       # 虎的比例

        # 按不同比例混合猫和虎的图像
        merge_image = cv2.addWeighted(
            cat, alpha, tiger, beta, 0.0)  # 0.0表示gamma，即亮度，0表示不调整

        if show_image:
            cv2.imshow(f'cat2tiger_{i}', merge_image)

        save_path = os.path.join(save_image_path, f'cat2tiger_{i}.png')
        # 保存图片
        cv2.imwrite(save_path, merge_image)

        if show_image:
            cv2.waitKey(0)

    if show_image:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()  # 生成猫到虎的序列图片
    image2video(save_image_path)  # 将图片保存成视频
