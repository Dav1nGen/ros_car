import cv2
import yaml
import numpy as np

qrcoder = cv2.QRCodeDetector()


def get_params():
    with open("camera_params.yaml", "r") as file:
        camera_params = yaml.safe_load(file)
        camera_matrix = np.array(camera_params["camera_matrix"]["data"]).reshape((3, 3))
        dist_coeffs = np.array(camera_params["dist_coeffs"]["data"])
        qrcode_size = camera_params["qrcode_size"]
    return camera_matrix, dist_coeffs, qrcode_size


def qrcode_detect(src_img, show_img: bool = True) -> bool:
    dectect_success, points = qrcoder.detect(src_img)
    if dectect_success:
        cv2.drawContours(src_img, [np.int32(points)], 0, (0, 0, 255), 2)
    if show_img:
        cv2.imshow("result", src_img)
        cv2.waitKey(0)

    return dectect_success, points


def qrcode_solve(qrcode_size, points_2d, camera_matrix, dist_coeffs):
    points_3d = np.array(
        [
            [0, 0, 0],
            [qrcode_size, 0, 0],
            [qrcode_size, qrcode_size, 0],
            [0, qrcode_size, 0],
        ],
        dtype=np.float32,
    )
    success, rvec, tvec = cv2.solvePnP(points_3d, points_2d, camera_matrix, dist_coeffs)
    return rvec, tvec


if __name__ == "__main__":
    camera_matrix, dist_coeffs, qrcode_size = get_params()
    dectect_success, points_2d = qrcode_detect(cv2.imread("/home/davi/下载/1.jpg"))
    if dectect_success:
        print(qrcode_solve(qrcode_size, points_2d, camera_matrix, dist_coeffs))
