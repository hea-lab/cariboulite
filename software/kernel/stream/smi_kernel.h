#include <linux/ioctl.h>

#define ACTIVATE_STREAM_TX _IOW('a',0,struct stream_config)
#define ACTIVATE_STREAM_RX _IOW('a',1,struct stream_config)

#define GET_METADATA _IOR('a',2,struct md)

#define SETUP_STREAM_TX _IOR('a',3,struct setup_stream)
#define SETUP_STREAM_RX _IOR('a',4,struct setup_stream)

#define CLOSE_STREAM_TX _IOR('a',5,struct setup_stream)
#define CLOSE_STREAM_RX _IOR('a',6,struct setup_stream)

#define DEACTIVATE_STREAM_TX _IOW('a',7,struct stream_config)
#define DEACTIVATE_STREAM_RX _IOW('a',8,struct stream_config)

struct setup_stream {
	bool is_rx;
};

struct stream_config {
	uint64_t activation_time; 
	int flags; 
	size_t num_elements; 
};

struct md {
	uint64_t timestamp; 
};
