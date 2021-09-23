/**
 * @brief simple utility to synchronize two messages
 **/
#ifndef BVS_UTILS_MESSAGE_SYNC_H_
#define BVS_UTILS_MESSAGE_SYNC_H_

#include <functional>
#include <queue>

namespace bvs_utils {

template<typename MessageA, typename MessageB>
class MessageSync {
public:
    typedef std::function<void(const MessageA&, const MessageB&)> Callback;

    /**
     * @brief creates a message syncer
     * @param time_sensitivity_s time (in senconds) messages must be within to be considered synced
     **/
    MessageSync(double time_sensitivity_s):
        time_sensitivity_s_(time_sensitivity_s)
    {};

    /**
     * @brief add a message of type MessageA
     * @param stamp the timestamp of message
     * @param message a message of type MessageA to sync with MessagB
     **/
    void addMessage(const double& stamp, const MessageA& message) {
        message_a_queue_.push({
            stamp,
            message
        });
        processQueue();
    }

    /**
     * @brief add a message of type MessageB
     * @param stamp the timestamp of message
     * @param message a message of type MessageB to sync with MessagA
     **/
    void addMessage(const double& stamp, const MessageB& message) {
        message_b_queue_.push({
            stamp,
            message
        });
        processQueue();
    }

    /**
     * @brief used to register a callback whenever messages are synced
     * @param callback the callback to use whenever messages are synced
     **/
    void registerCallback(Callback callback) {
        callback_ = callback;
    }

    /**
     * @brief used to get synced message pairs from the queue (instead of callbacks)
     * @param message_a[out] message of type MessageA synced to message_b
     * @param message_b[out] message of type MessageB synced to message_a
     * @return true if there are synced messages that message_a and message_b
     *  are set to, otherwise false
     **/
    bool processMessages(MessageA& message_a, MessageB& message_b) {
        while(true) {
            // We must have messages in both queues to be synced
            if(message_a_queue_.size() == 0 || message_b_queue_.size() == 0) {
                return false;
            }
            auto message_a_front = message_a_queue_.front();
            auto message_b_front = message_b_queue_.front();

            // Check the front of the queue and see if they are synced
            if(std::abs(message_a_front.first - message_b_front.first) < time_sensitivity_s_) {
                message_a = message_a_front.second;
                message_b = message_b_front.second;
                message_a_queue_.pop();
                message_b_queue_.pop();
                return true;
            }
            // At this point we know that the timestamps are not synced
            // so we should delete the older message to try to sync that
            // queue to the newer messages
            if(message_a_front.first < message_b_front.first) {
                message_a_queue_.pop();
            } else {
                message_b_queue_.pop();
            }
        }
        return false;
    }

private:
    /**
     * @brief called by addMessage to process the queue
     *  and call the callback when it exists
     **/
    void processQueue() {
        if(callback_ == nullptr) {
            return;
        }
        MessageA message_a;
        MessageB message_b;
        while(processMessages(message_a, message_b)) {
            callback_(message_a, message_b);
        }
    }

    //! Stores recieved messages of type MessageA and their timestampe
    std::queue<std::pair<double, MessageA>> message_a_queue_;
    //! Stores recieved messages to type MessageB and their timestamp
    std::queue<std::pair<double, MessageB>> message_b_queue_;
    //! Time difference to consider messages to be synced if they are within
    double time_sensitivity_s_;
    //! Callback to process synced messages on
    Callback callback_;

};

} /* namespace bvs_utils */



#endif /* BVS_UTILS_MESSAGE_SYNC_H_ */