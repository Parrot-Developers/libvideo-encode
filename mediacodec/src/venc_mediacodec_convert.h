/**
 * Copyright (c) 2017 Parrot Drones SAS
 */

#define ARRAY_SIZE(x) (sizeof(x) / sizeof(*x))


static struct {
	media_status_t status;
	const char *status_str;
} media_status_map[] = {
	{AMEDIA_OK, "OK"},
	{AMEDIACODEC_ERROR_INSUFFICIENT_RESOURCE,
	 "ERROR_INSUFFICIENT_RESOURCE"},
	{AMEDIACODEC_ERROR_RECLAIMED, "ERROR_RECLAIMED"},
	{AMEDIA_ERROR_BASE, "ERROR_BASE"},
	{AMEDIA_ERROR_UNKNOWN, "ERROR_UNKNOWN"},
	{AMEDIA_ERROR_MALFORMED, "ERROR_MALFORMED"},
	{AMEDIA_ERROR_UNSUPPORTED, "ERROR_UNSUPPORTED"},
	{AMEDIA_ERROR_INVALID_OBJECT, "ERROR_INVALID_OBJECT"},
	{AMEDIA_ERROR_INVALID_PARAMETER, "ERROR_INVALID_PARAMETER"},
	{AMEDIA_ERROR_INVALID_OPERATION, "ERROR_INVALID_OPERATION"},
	{AMEDIA_ERROR_END_OF_STREAM, "ERROR_END_OF_STREAM"},
	{AMEDIA_ERROR_IO, "ERROR_IO"},
	{AMEDIA_ERROR_WOULD_BLOCK, "ERROR_WOULD_BLOCK"},
	{AMEDIA_DRM_ERROR_BASE, "DRM_ERROR_BASE"},
	{AMEDIA_DRM_NOT_PROVISIONED, "DRM_NOT_PROVISIONED"},
	{AMEDIA_DRM_RESOURCE_BUSY, "DRM_RESOURCE_BUSY"},
	{AMEDIA_DRM_DEVICE_REVOKED, "DRM_DEVICE_REVOKED"},
	{AMEDIA_DRM_SHORT_BUFFER, "DRM_SHORT_BUFFER"},
	{AMEDIA_DRM_SESSION_NOT_OPENED, "DRM_SESSION_NOT_OPENED"},
	{AMEDIA_DRM_TAMPER_DETECTED, "DRM_TAMPER_DETECTED"},
	{AMEDIA_DRM_VERIFY_FAILED, "DRM_VERIFY_FAILED"},
	{AMEDIA_DRM_NEED_KEY, "DRM_NEED_KEY"},
	{AMEDIA_DRM_LICENSE_EXPIRED, "DRM_LICENSE_EXPIRED"},
	{AMEDIA_IMGREADER_ERROR_BASE, "IMGREADER_ERROR_BASE"},
	{AMEDIA_IMGREADER_NO_BUFFER_AVAILABLE, "IMGREADER_NO_BUFFER_AVAILABLE"},
	{AMEDIA_IMGREADER_MAX_IMAGES_ACQUIRED, "IMGREADER_MAX_IMAGES_ACQUIRED"},
	{AMEDIA_IMGREADER_CANNOT_LOCK_IMAGE, "IMGREADER_CANNOT_LOCK_IMAGE"},
	{AMEDIA_IMGREADER_CANNOT_UNLOCK_IMAGE, "IMGREADER_CANNOT_UNLOCK_IMAGE"},
	{AMEDIA_IMGREADER_IMAGE_NOT_LOCKED, "IMGREADER_IMAGE_NOT_LOCKED"},
};


static inline const char *media_status_to_str(media_status_t status)
{
	unsigned int i;
	for (i = 0; i < ARRAY_SIZE(media_status_map); i++) {
		if (media_status_map[i].status == status)
			return media_status_map[i].status_str;
	}
	return "UNKNOWN";
}


static inline bool convert_h264_profile(unsigned int p, enum mc_h264_profile *o)
{
	switch (p) {
	case H264_PROFILE_MAIN:
		*o = MC_H264_PROFILE_MAIN;
		return true;
	default:
		return false;
	}
}


static inline bool convert_h264_level(unsigned int l, enum mc_h264_level *o)
{
	switch (l) {
	case 40:
		*o = MC_H264_LEVEL_4;
		return true;
	default:
		return false;
	}
}


static inline bool convert_h265_profile(unsigned int p, enum mc_h265_profile *o)
{
	switch (p) {
	case 1:
		*o = MC_H265_PROFILE_MAIN;
		return true;
	default:
		return false;
	}
}


static inline bool convert_h265_level(unsigned int l, enum mc_h265_level *o)
{
	switch (l) {
	case 40:
		*o = MC_H265_MAIN_TIER_LEVEL4;
		return true;

	default:
		return false;
	}
}
