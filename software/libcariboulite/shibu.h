#include <linux/ioctl.h>

#define ACTIVATE_STREAM _IOW('a',1,struct stream_config)
#define GET_METADATA _IOR('a',2,struct md)
#define SETUP_STREAM _IOR('a',3,struct setup_stream)

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

