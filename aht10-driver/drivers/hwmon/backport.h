#define HWMON_CHANNEL_INFO(stype, ...)	\
	(&(struct hwmon_channel_info) {	\
		.type = hwmon_##stype,	\
		.config = (u32 []) {	\
			__VA_ARGS__, 0	\
		}			\
	})
