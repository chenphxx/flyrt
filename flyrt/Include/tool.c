#include "tool.h"

// 计算开根号的倒数
float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));

    return y;
}

// 原始值转换为弧度值
void get_radian(float *gx, float *gy, float *gz, float *gx_rad, float *gy_rad, float *gz_rad)
{
    *gx_rad = (float)(*gx * RawData_to_Radian);
    *gy_rad = (float)(*gz * RawData_to_Radian);
    *gz_rad = (float)(*gx * RawData_to_Radian);
}

// 快速排序辅助函数：确定一个元素的位置
int partition(int16_t *arr, int low, int high)
{
    int16_t pivot = arr[low];

    while (low < high)
    {
        while (low < high && arr[high] >= pivot) high--;
        arr[low] = arr[high];
        while (low < high && arr[low] <= pivot) low++;
        arr[high] = arr[low];
    }

    arr[low] = pivot;
    return low;
}

// 快速排序主函数
void quick_sort(int16_t *arr, int low, int high)
{
    if (low < high)
    {
        int mid = partition(arr, low, high);
        
        quick_sort(arr, low, mid - 1);
        quick_sort(arr, mid + 1, high);
    }
}
