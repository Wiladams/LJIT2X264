local ffi = require "ffi"

local libx264 = ffi.load("libx264-120.dll")
local x264 = require "x264"

local width = 640
local height = 480
local err = -1

local x264_param = ffi.new"x264_param_t"

-- Fill the structure with defaults
--x264.lib.x264_param_default(x264_param);


local err = x264.lib.x264_param_default_preset(x264_param, "veryfast", "zerolatency");

assert(0 == err)

local fps = 60
x264_param.i_threads = 8;
x264_param.i_width = width;
x264_param.i_height = height;
x264_param.i_fps_num = fps;		-- fps;
x264_param.i_fps_den = 1;
-- Intra refres:
x264_param.i_keyint_max = fps;	-- fps;
x264_param.b_intra_refresh = 1;
-- Rate control:
x264_param.rc.i_rc_method = x264.RC_CRF;
x264_param.rc.f_rf_constant = 25;
x264_param.rc.f_rf_constant_max = 35;
-- For streaming:
x264_param.b_repeat_headers = 1;
x264_param.b_annexb = 1;

err = x264.lib.x264_param_apply_profile(x264_param, "baseline");
assert(0==err);

local x264_encoder = x264.encoder_open(x264_param);


