import numpy as np
import matplotlib.pyplot as plt
import bound_with_delay as bound_lib


def readFile(filename):
       data_path = '/home/paper164/ros2_dashing/src/synchronizer/results/'
       f = open(data_path + filename,'r',encoding='utf-8') # 打开返回文件对象f

       data_list = []
       for line in f:
              data_list.append(float(line))

       return data_list

def Average(lst):
    return sum(lst) / len(lst)

def get_observed_periods(timestamp_list_path):

    observed_periods = []

    for i in range(0,10,2):
        for j in range(1,7):
            time = readFile(timestamp_list_path+str(i)+'/topic'+str(j)+'_timestamp.txt')
            observed_periods.append(Average(time))

    return observed_periods

observed_timestamp_list_path = \
            ['evaluation/varied_period_factor/varied_period_factor_1.0',
             'evaluation/varied_period_factor/varied_period_factor_1.2',
             'evaluation/varied_period_factor/varied_period_factor_1.4',
             'evaluation/varied_period_factor/varied_period_factor_1.6',
             'evaluation/varied_period_factor/varied_period_factor_1.8'
             ]

def get_both_bounds(seq_num, channel_num, delay_best, delay_worst):
    periods_num = 0
    periods_list = []
    for k in range(channel_num):
        periods_of_topic = readFile(observed_timestamp_list_path[seq_num]+'/topic'+str(k+1)+'_timestamp.txt')
        periods_list.append(periods_of_topic)
        periods_num = len(periods_of_topic)

    ros_bound_sum = 0
    cyber_bound_sum = 0
    for i in range(periods_num):
        periods = []
        for j in range(channel_num):
            periods.append(periods_list[j][i])
        each_ros_bound, each_cyber_bound = bound_lib.cal_bound(periods, delay_best, delay_worst)
        ros_bound_sum += each_ros_bound
        cyber_bound_sum += each_cyber_bound

    return ros_bound_sum/periods_num, cyber_bound_sum/periods_num

def plot_clustered_stacked(dfall, labels=None, title="multiple stacked bar plot",  H="/", **kwargs):
    """Given a list of dataframes, with identical columns and index, create a clustered stacked bar plot.
labels is a list of the names of the dataframe, used for the legend
title is a string for the title of the plot
H is the hatch used for identification of the different dataframe"""

    n_df = len(dfall)
    n_col = len(dfall[0].columns)
    n_ind = len(dfall[0].index)
    axe = plt.subplot(111)

    for df in dfall : # for each data frame
        axe = df.plot(kind="bar",
                      linewidth=0,
                      stacked=True,
                      ax=axe,
                      legend=False,
                      grid=False,
                      **kwargs)  # make bar plots

    h,l = axe.get_legend_handles_labels() # get the handles we want to modify
    for i in range(0, n_df * n_col, n_col): # len(h) = n_col * n_df
        for j, pa in enumerate(h[i:i+n_col]):
            for rect in pa.patches: # for each index
                rect.set_x(rect.get_x() + 1 / float(n_df + 1) * i / float(n_col))
                rect.set_hatch(H * int(i / n_col)) #edited part
                rect.set_width(1 / float(n_df + 1))

    axe.set_xticks((np.arange(0, 2 * n_ind, 2) + 1 / float(n_df + 1)) / 2.)
    ax.set_xticklabels([])
    axe.set_title(title)

    # Add invisible data to add another legend
    n=[]
    for i in range(n_df):
        n.append(axe.bar(0, 0, color="gray", hatch=H * i))

    l1 = axe.legend(h[:n_col], l[:n_col], loc=[1.01, 0.5])
    if labels is not None:
        l2 = plt.legend(n, labels, loc=[1.01, 0.1])
    axe.add_artist(l1)
    return axe

file_list = ['evaluation/varied_period_factor/varied_period_factor_1.0/timestamp_lat.txt',
             'evaluation/varied_period_factor/varied_period_factor_1.0/timestamp_mdl.txt',
             'evaluation/varied_period_factor/varied_period_factor_1.2/timestamp_lat.txt',
             'evaluation/varied_period_factor/varied_period_factor_1.2/timestamp_mdl.txt',
             'evaluation/varied_period_factor/varied_period_factor_1.4/timestamp_lat.txt',
             'evaluation/varied_period_factor/varied_period_factor_1.4/timestamp_mdl.txt',
             'evaluation/varied_period_factor/varied_period_factor_1.6/timestamp_lat.txt',
             'evaluation/varied_period_factor/varied_period_factor_1.6/timestamp_mdl.txt',
             'evaluation/varied_period_factor/varied_period_factor_1.8/timestamp_lat.txt',
             'evaluation/varied_period_factor/varied_period_factor_1.8/timestamp_mdl.txt'
             ]

labels = ['1.0', '1.2', '1.4', '1.6', '1.8']
# labels = ['CyberRT', 'ROS', 'CyberRT', 'ROS', 'CyberRT', 'ROS', 'CyberRT', 'ROS', 'CyberRT', 'ROS']
# x = np.arange(len(labels))  # the label locations
# x=10
width = 0.3  # the width of the bars
ChanNum = 6
# periods = get_observed_periods(subed_timestamp_list_path)
# print(periods)
# print(get_observed_periods(pubed_timestamp_list))

rt_worst_case = []
ros_worst_case = []
rt_bound_list = []
ros_bound_list = []

for i in range(0, len(file_list), 2):
       rt_data = readFile(file_list[i])
       ros_data = readFile(file_list[i+1])
       rt_worst_case.append(Average(rt_data))
       ros_worst_case.append(Average(ros_data))

delay_worst = []
delay_best = []
for j in range(len(labels)):
   for k in range(ChanNum):
       delay_worst.append(40)
       delay_best.append(1)
   ros_bound, cyber_bound = get_both_bounds(j, ChanNum, delay_best, delay_worst)
   rt_bound_list.append(cyber_bound)
   ros_bound_list.append(ros_bound)

fig, ax = plt.subplots()

plt.tick_params(
    axis='x',          # changes apply to the x-axis
    which='both',      # both major and minor ticks are affected
    bottom=False,      # ticks along the bottom edge are off
    top=False,         # ticks along the top edge are off
    labelbottom=True) # labels along the bottom edge are off

ax.yaxis.grid(True, linestyle='-', which='major',
                # color='lightgrey',
               alpha=0.5)

# lightsteelblue
ros_bound_pos = np.arange(0.8,10.8,2)
rects_ros_bound = ax.bar(ros_bound_pos + width/2, ros_bound_list, width, color='brown', label='ROS-B')
# cornflowerblue
ros_worst_case_pos = np.arange(0.8,10.8,2)
rects_ros_worst = ax.bar(ros_worst_case_pos - width/2, ros_worst_case, width, color='rosybrown', label='ROS-O')

rt_bound_pos = np.arange(0,10,2)
rects_rt_bound = ax.bar(rt_bound_pos + width/2, rt_bound_list, width, color='steelblue', label='CyberRT-B')

rt_worst_case_pos = np.arange(0,10,2)
rects_rt_worst = ax.bar(rt_worst_case_pos - width/2, rt_worst_case, width, color='lightsteelblue', label='CyberRT-O')


ax.tick_params(axis='both', which='major', labelsize=15)
# Add some text for labels, title and custom x-axis tick labels, etc.
ax.set_axisbelow(True)
# ax.set_title('Comparison of Time Disparity with Varied Tw/Tb')
ax.set_xlabel('$T^W_i/T^B_i$',fontsize=20)
ax.set_ylabel('Time Disparity (ms)',fontsize=20)

delay_label_pos = np.arange(0.4,10.4,2)
ax.set_xticks(delay_label_pos)
ax.set_xticklabels(labels)
ax.legend(fontsize=13,loc='lower right')

# ax.bar_label(rects1, padding=3)
# ax.bar_label(rects2, padding=3)

fig.tight_layout()

# plt.show()

# scatter_fig = fig.get_figure()
plt.savefig('his_fig_varied_period_factor', dpi=400)

print("Generate figure (c) successfully. ")