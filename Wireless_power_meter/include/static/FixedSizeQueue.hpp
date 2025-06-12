/*
 * @LastEditors: qingmeijiupiao
 * @Description: 定长队列
 * @Author: qingmeijiupiao
 * @LastEditTime: 2024-11-17 23:37:28
 */
#ifndef FixedSizeQueue_HPP
#define FixedSizeQueue_HPP



#include <queue>
template <typename T, size_t N>
class FixedSizeQueue {
private:
    std::queue<T> queue;
    size_t capacity;

    T* array = new T[N];
    T max;
    int max_index;
    T min;
    int min_index;
    T total_value;
    T average;

public:
    FixedSizeQueue() : capacity(N) {}

    void push(const T& value) {
        // 如果队列已满，将队首元素出队
        if (queue.size() == capacity) {
            T pop_value = queue.front();
            queue.pop();// 出队
           
            //更新总值
            total_value -= pop_value;

            
            if(max_index>=N){// 如果最大值已经出队，查找当前队列最大值
                std::queue<T> copy = queue;//复制队列
                max=copy.front();//队首元素
                int now_index=0;//当前索引
                max_index=0;//队首元素为最大值情况
                while (!copy.empty()) {//遍历队列
                    T _value = copy.front();
                    copy.pop();
                    if (_value > max) {//更新最大值
                        max = value;//更新最大值
                        max_index = now_index;//更新最大值索引
                        break;
                    }
                    now_index++;//更新索引
                }
            }
            if(min_index>=N){// 如果最小值已经出队，查找当前队列最小值
                std::queue<T> copy = queue;//复制队列
                min=copy.front();//队首元素
                int now_index=0;//当前索引
                min_index=0;//队首元素为最小值情况
                while (!copy.empty()) {//遍历队列
                    T _value = copy.front();
                    copy.pop();
                    if (_value < min) {//更新最小值
                        min = _value;//更新最小值
                        min_index = now_index;//更新最小值索引
                        break;
                    }
                    now_index++;//更新索引
                }
            }

        }
        if(queue.empty()){
            max=value;
            min=value;
            max_index=0;
            min_index=0;
        }
        if(value>max){
            max=value;
            max_index=0;
        }
        if(value<min){
            min=value;
            min_index=0;
        }

        total_value += value;
        average = total_value / queue.size();
        queue.push(value); // 添加新元素
        max_index++;
        min_index++;
    }
    T back(){
        return queue.back();
    }

    bool isEmpty(){
        return queue.empty();
    }

    size_t size(){
        return queue.size();
    }
    std::queue<T> copy(){
        return queue;
    }
    T get_average(){
        return total_value / queue.size();
    }

    T get_max(){
        return max;
    }

    T get_min(){
        return min;
    }
    T* toArray(){
        std::queue<T> copy = queue;
        size_t index = 0;
        while (!copy.empty()) {
            array[index++] = copy.front();
            copy.pop();
        }
        if (index < N){
            for (size_t i = index; i < N; i++)
            {
                array[i] = 0;
            }
        }
        return array;
    }
};
#endif // FixedSizeQueue_HPP
