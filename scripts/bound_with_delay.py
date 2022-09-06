import heapq

def cal_sum(values, k):
    sum = 0
    for i in range(k):
        sum += values[i]

    return sum

def cal_ros_bound(T):
    max_value = 0
    for i in range(2, len(T) + 1):  # 1, 2
        max_n = heapq.nlargest(i - 1, T)
        avg = cal_sum(max_n, i - 1) / i
        if avg > max_value:
            max_value = avg
    return max_value

def cal_cyberrt_bound(T, D_best, D_worst):
    max1 = 0
    for i in range(1, len(T)):
        sum = T[i] + D_worst[i]
        if sum > max1:
            max1 = sum
    item1 = max1 - D_best[0]

    min2 = float('inf')
    for i in range(1, len(D_best)):
        if D_best[i] < min2:
            min2 = D_best[i]
    item2 = D_worst[0] - min2

    return max(item1, item2)


def cal_bound(T, D_best, D_worst):
    ros_bound = cal_ros_bound(T)  # we should calculate 2<=n<=N

    cyberrt_bound = cal_cyberrt_bound(T, D_best, D_worst)

    return ros_bound, cyberrt_bound