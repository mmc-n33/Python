# -*- coding: utf-8 -*-
"""
Created on Sun Jan 27 16:27:15 2019

@author: Mingchen Mao
"""

"""lecture 4 example"""
"""
x = np.matrix([[-3, 9, 1], [-2, 4, 1], [-1, 1, 1], [0, 0, 1], [1, 1, 1], [3, 9, 1]])
y = np.transpose([1, 1, -1, -1, -1, 1])
omega = np.zeros((3, 1))
alpha = 0.1

for j in range(10):
    he = np.zeros((3, 1))
    for i in range(len(y)):
        he += np.transpose(x[i]) * (1 / (1 + np.exp(-x[i] * omega)) - y[i])
    omega -= alpha * he

"""

"""given code"""
"""
if __name__ == '__main__':
    folder = "trainset"
    my_detector = BarrelDetector()
    for filename in os.listdir(folder):
        # read one test image
        img = cv2.imread(os.path.join(folder,filename))
        cv2.imshow('image', img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        #Display results:
        #(1) Segmented images
        #     mask_img = my_detector.segment_image(img)
        #(2) Barrel bounding box
        #    boxes = my_detector.get_bounding_box(img)
        #The autograder checks your answers to the functions segment_image() and get_bounding_box()
        #Make sure your code runs as expected on the testset before submitting to Gradescope

"""

        

