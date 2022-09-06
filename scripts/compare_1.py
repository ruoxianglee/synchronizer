# Comparing the synchronization results
# from Approximate Time Algorithm implementation and our model implementation.

import filecmp
import datetime

path = "/home/paper164/ros2_dashing/src/synchronizer/results/validation_timer/"

# path_list = [base_path+"validation_rosbag/", base_path+"validation_timer/"]
# path_list = [base_path+"validation_timer/"]

# test_channel_num = ["channel_num/channel_2", "channel_num/channel_3", "channel_num/channel_4", "channel_num/channel_5", "channel_num/channel_6", "channel_num/channel_7", "channel_num/channel_8", "channel_num/channel_9"]
test_channel_num = ["channel_num/channel_3", "channel_num/channel_4", "channel_num/channel_5", "channel_num/channel_6", "channel_num/channel_7", "channel_num/channel_8", "channel_num/channel_9"]
test_varied_periods = ["varied_periods/varied_period_lower_10", "varied_periods/varied_period_lower_20", "varied_periods/varied_period_lower_30", "varied_periods/varied_period_lower_40", "varied_periods/varied_period_lower_50"]
test_varied_period_factor = ["varied_period_factor/varied_period_factor_1.0", "varied_period_factor/varied_period_factor_1.2", "varied_period_factor/varied_period_factor_1.4", "varied_period_factor/varied_period_factor_1.6", "varied_period_factor/varied_period_factor_1.8"]

# test_list = [test_sensor_data, test_channel_num, test_varied_periods, test_varied_period_factor]
test_list = [test_channel_num, test_varied_periods, test_varied_period_factor]

test_result = True

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

time1 = datetime.datetime.now()
print(time1)