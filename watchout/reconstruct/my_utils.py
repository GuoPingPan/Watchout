def bbox_rel(*xyxy):
    """" Calculates the relative bounding box from absolute pixel values. """
    bbox_left = min([xyxy[0].item(), xyxy[2].item()])
    bbox_top = min([xyxy[1].item(), xyxy[3].item()])
    bbox_w = abs(xyxy[0].item() - xyxy[2].item())
    bbox_h = abs(xyxy[1].item() - xyxy[3].item())
    x_c = (bbox_left + bbox_w / 2)
    y_c = (bbox_top + bbox_h / 2)
    w = bbox_w
    h = bbox_h
    return x_c, y_c, w, h

palette = (2 ** 11 - 1, 2 ** 15 - 1, 2 ** 20 - 1)


def compute_color_for_labels(label):
    """
    Simple function that adds fixed color depending on the class
    """
    color = [int((p * (label ** 2 - label + 1)) % 255) for p in palette]
    return tuple(color)


def showdepth(boxes, depth):
    for box in boxes:
        x1, y1, x2, y2 = [int(i) for i in box]
        for u in range(x1, x2):
            for v in range(y1, y2):
                print(depth[v, u] * 0.001)


def goodenbox(bbox_xyxy):
    x1, y1, x2, y2 = [int(i) for i in bbox_xyxy]
    w = x2 - x1
    h = y2 - y1
    # 黄金比例切割背景
    import math
    u1 = math.ceil(x1 + 0.382 * w)
    u2 = math.ceil(x1 + 0.618 * w)
    v1 = math.ceil(y1 + 0.382 * h)
    v2 = math.ceil(y1 + 0.618 * h)
    return [u1, v1, u2, v2]


# 注意 offset 光心偏移
def draw_boxes(img, bbox, identities=None, offset=(0, 0)):
    for i, box in enumerate(bbox):
        x1, y1, x2, y2 = [int(i) for i in box]
        x1 += offset[0]
        x2 += offset[0]
        y1 += offset[1]
        y2 += offset[1]

        x1, y1, x2, y2 = goodenbox(x1, y1, x2, y2)

        # box text and bar
        id = int(identities[i]) if identities is not None else 0
        color = compute_color_for_labels(id)
        label = '{}{:d}'.format("", id)
        t_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_PLAIN, 2, 2)[0]
        cv2.rectangle(img, (x1, y1), (x2, y2), color, 3)
        # cv2.rectangle(
        #     img, (x1, y1), (x1 + t_size[0] + 3, y1 + t_size[1] + 4), color, -1)
        # cv2.putText(img, label, (x1, y1 +
        #                          t_size[1] + 4), cv2.FONT_HERSHEY_PLAIN, 2, [255, 255, 255], 2)
    return img


def kafilter(cxp, cyp, vx, vy, cx, cy, dt):
    F = np.identity(4)
    for i in range(2):
        F[i, i + 2] = dt

    pw = 1. / 20
    vw = 1. / 160

    mean = [cxp, cyp, vx, vy]

    # std的存在只是为了初始化协方差
    std = [2 * pw * cxp,
           2 * pw * cyp,
           10 * vw * vx,
           10 * vw * vy]
    covariance = np.diag(np.square(std))

    # 真正的std的计算
    std_true = [pw * cxp,
                pw * cyp,
                vw * vx,
                vw * vy]

    # 运动预测方程的误差
    Q = np.diag(np.square(std_true))
    # x' = Fx
    mean = np.dot(F, mean)
    # P' = FPF'+Q
    covariance = np.linalg.multi_dot((F, covariance, F.transpose()) + Q)

    pre = mean[:2]
    pvar = covariance[:2, :2]

    z = [cx, cy]
    z_std = [pw * cx,
             pw * cy]
    R = np.diag(np.square(z_std))

    import scipy.linalg as LA
    try:
        cho, lower = LA.cho_factor(
            (pvar + R).transpose(), lower=True, check_finite=False
        )
        K = LA.cho_solve(
            (cho, lower), pvar.transpose(), check_finite=False
        ).transpose()
        # x' = x +K(z-Hx)
        pose = pre + np.dot(K, z - pre)
        # P' = P - K'HP
        # new_covariance = covariance - np.linalg.multi_dot((
        # K, pvar, K.transpose()))

        return pose[0], pose[1]
    except:
        return cx, cy
