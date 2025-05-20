import cv2

img = cv2.imread("snapshot.jpg")
if img is not None:
    cv2.imshow("Snapshot", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
else:
    print("ERROR loading image")
