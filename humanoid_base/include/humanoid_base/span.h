#ifndef __HUMANOID_SPAN_H__
#define __HUMANOID_SPAN_H__

#include <cstring>

namespace humanoid {

template <class DataType, class MessageType>
class ZeroCopyMessage {
public:
    inline ZeroCopyMessage(const DataType *d) : _data(d) {}

    inline ZeroCopyMessage(ZeroCopyMessage &&) = delete;

    inline ZeroCopyMessage(const ZeroCopyMessage &) = delete;

    inline const MessageType &get_message() const {
        return *reinterpret_cast<const MessageType *>(_data);
    }

protected:
    const DataType *_data;
};

}  // namespace humanoid

#endif  // __HUMANOID_SPAN_H__
