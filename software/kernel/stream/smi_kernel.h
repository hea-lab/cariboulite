#include <linux/ioctl.h>

#define ACTIVATE_STREAM_TX _IOW('a',0,struct stream_config)
#define ACTIVATE_STREAM_RX _IOW('a',1,struct stream_config)

#define GET_METADATA _IOR('a',2,struct md)

#define SETUP_STREAM_TX _IO('a',3)
#define SETUP_STREAM_RX _IO('a',4)

#define CLOSE_STREAM_TX _IO('a',5)
#define CLOSE_STREAM_RX _IO('a',6)

#define DEACTIVATE_STREAM_TX _IO('a',7)
#define DEACTIVATE_STREAM_RX _IO('a',8)

struct stream_config {
	uint64_t activation_time; 
	int flags; 
	size_t num_elements; 
};

struct md {
	uint64_t timestamp; 
};
