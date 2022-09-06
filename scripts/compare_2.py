# Comparing the synchronization results
# from Approximate Time Algorithm implementation and our model implementation.

import filecmp

test_result = True

path = "/home/paper164/ros2_dashing/src/synchronizer/results/validation_rosbag/"
test_sensor_data = ["cam_imu", "cam_lidar", "imu_lidar", "cam_imu_lidar"]
test_list = [test_sensor_data]

# for path in path_list:
for test in test_list:
    # Comparing complete file at once
    # Deep comparison
    for i in range(len(test)):
        file1 = path + test[i] + "/timestamp_alg.txt"
        file2 = path + test[i] + "/timestamp_mdl.txt"
        result = filecmp.cmp(file1, file2, shallow=False)
        test_result = result 
        # print("Comparision result for", test[i], ":", result)

    # Comparing files line by line
    for j in range(len(test)):
        # reading files
        f1 = open(path + test[j] + "/timestamp_alg.txt")
        f2 = open(path + test[j] + "/timestamp_mdl.txt")
        i = 0

        # print("Comparision result for", test[j], " ...")
        for line1 in f1:
            i += 1
            # value_list1 = line1.split()

            for line2 in f2:
                # matching line1 from both files
                if line1 != line2:
                    test_result = False
                    # print("Line ", i, ":")
                    # print("\tFile 1:", line1, end='')
                    # print("\tFile 2:", line2, end='')
                break

        # closing files
        f1.close()
        f2.close()
        
if test_result == True:
    print("Comparision test passed successfully. ")

if test_result == False:
    print("Comparision test failed. ")