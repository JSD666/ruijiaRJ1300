#ifndef EVENT_H
#define EVENT_H

#include <future>
#include <atomic>
#include <chrono>
#include <mutex>
#include <cassert>

// 泛型同步事件，仅可用于两个线程间的同步，不支持多个线程等待
// 可以在调用notify()的时候传值，wait()之后可以调用get()获取传递的值
// Example:
//
//      BoolEvent something_happened_flag;
//
//  In thread waiter:
//      // 等待事件发生，等待时间为10s
//      const bool notified = something_happened_flag.wait_for(std::chrono::seconds(10));
//      if (!notified) {
//          cout << "Time out!" << endl;
//      } else { // 等待成功，获取通知值
//          const bool happened = something_happened_flag.get();
//          // ...
//      }
//
//  In thread notifier:
//      // 侦测到事件发生
//      if (...) {
//          something_happened_flag.notify(true);
//      }
//
template<typename T>
class Event
{
public:
    // 析构函数，自动释放等待资源
    inline ~Event() { notify(); }

    // 判断该事件是否有等待者
    inline bool isWaiting() const { return is_waiting_; }

    // 若事件有等待等，向其传递参数并通知它事件满足
    void notify(const T& value = T())
    {
        // 有等待者才通知
        if (is_waiting_)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (is_waiting_)  // double-check，提高上锁性能
            {
                is_waiting_ = false;
                promise_data_.set_value(value);
            }
        }
    }

    // 无超时阻塞等待事件满足
    void wait()
    {
        assert (!is_waiting_ && "Event<T>不支持多线程同时等待");

        {
            std::lock_guard<std::mutex> lock(mutex_);
            promise_data_ = std::promise<T>();
            is_waiting_ = true;
        }

        auto future_data = promise_data_.get_future();
        future_data.wait();

        {
            std::lock_guard<std::mutex> lock(mutex_);
            is_waiting_ = false;
            data_ = future_data.get();
        }
    }

    // 超时阻塞等待事件满足。若超时返回false，否则返回true
    template< class Rep, class Period >
    bool wait_for(const std::chrono::duration<Rep,Period>& timeout_duration)
    {
        assert (!is_waiting_ && "Event<T>不支持多线程同时等待");

        {
            std::lock_guard<std::mutex> lock(mutex_);
            promise_data_ = std::promise<T>();
            is_waiting_ = true;
        }

        auto future_data = promise_data_.get_future();
        const std::future_status status = future_data.wait_for(timeout_duration);

        {
            std::lock_guard<std::mutex> lock(mutex_);
            is_waiting_ = false;
        }

        if (status != std::future_status::ready) { return false; }

        {
            std::lock_guard<std::mutex> lock(mutex_);
            data_ = future_data.get();
        }

        return true;
    }

    // 获取notify()调用传递的参数，成功wait()之后值才是有效的
    inline const T& get() const { return data_; }

    // 获取notify()调用传递的参数，成功wait()之后值才是有效的
    inline T& get() { return data_; }
    std::string signal_device ="";
    int signal_seq = 1;
    int signal_value = 1;

private:
    std::atomic_bool is_waiting_ { false };
    std::promise<T> promise_data_;
    std::mutex mutex_;
    T data_;
};

using BoolEvent = Event<bool>;
using IntEvent = Event<int>;
using FloatEvent = Event<float>;
using DoubleEvent = Event<double>;

#endif // EVENT_H