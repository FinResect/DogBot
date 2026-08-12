#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <gst/gst.h>
#include <gst/rtsp-server/rtsp-server.h>
#pragma GCC diagnostic pop

#include <mutex>
#include <string>
#include <thread>

namespace dogbot_core::camera {

class RtspStreamNode : public rclcpp::Node {
public:
    explicit RtspStreamNode(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions())
        : Node("rtsp_stream_node", opts) {
        bool enabled = declare_parameter("enabled", true);

        if (!enabled) {
            RCLCPP_INFO(get_logger(), "RTSP streaming disabled");
            return;
        }

        port_        = declare_parameter("port", 8554);
        mount_point_ = declare_parameter("mount_point", "/cam");
        bitrate_     = declare_parameter("bitrate", 2000);
        width_       = declare_parameter("width", 640);
        height_      = declare_parameter("height", 480);
        fps_         = declare_parameter("fps", 30);

        sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/vision/following/image", 10,
            std::bind(&RtspStreamNode::image_callback, this, std::placeholders::_1));

        gst_init(nullptr, nullptr);
        start_rtsp_server();
    }

    ~RtspStreamNode() override {
        if (loop_) {
            g_main_loop_quit(loop_);
        }
        if (gst_thread_.joinable()) {
            gst_thread_.join();
        }
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            std::lock_guard<std::mutex> lock(frame_mutex_);
            current_frame_ = cv_ptr->image.clone();
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(get_logger(), "cv_bridge error: %s", e.what());
        }
    }

    static void need_data(GstElement* appsrc, guint /*unused*/, gpointer user_data) {
        auto* self = static_cast<RtspStreamNode*>(user_data);
        std::lock_guard<std::mutex> lock(self->frame_mutex_);

        if (self->current_frame_.empty()) {
            return;
        }

        int size          = self->current_frame_.total() * self->current_frame_.elemSize();
        GstBuffer* buffer = gst_buffer_new_allocate(nullptr, size, nullptr);

        GstMapInfo map;
        gst_buffer_map(buffer, &map, GST_MAP_WRITE);
        std::memcpy(map.data, self->current_frame_.data, size);
        gst_buffer_unmap(buffer, &map);

        GST_BUFFER_PTS(buffer)      = self->timestamp_;
        GST_BUFFER_DURATION(buffer) = gst_util_uint64_scale_int(1, GST_SECOND, self->fps_);
        self->timestamp_ += GST_BUFFER_DURATION(buffer);

        GstFlowReturn ret;
        g_signal_emit_by_name(appsrc, "push-buffer", buffer, &ret);
        gst_buffer_unref(buffer);

        if (ret != GST_FLOW_OK) {
            RCLCPP_WARN(self->get_logger(), "appsrc push-buffer returned %d", ret);
        }
    }

    static void
        media_configure(GstRTSPMediaFactory* /*factory*/, GstRTSPMedia* media, gpointer user_data) {
        auto* self           = static_cast<RtspStreamNode*>(user_data);
        GstElement* pipeline = gst_rtsp_media_get_element(media);
        GstElement* appsrc   = gst_bin_get_by_name(GST_BIN(pipeline), "mysrc");

        g_signal_connect(appsrc, "need-data", G_CALLBACK(need_data), self);

        GstCaps* caps = gst_caps_new_simple(
            "video/x-raw", "format", G_TYPE_STRING, "BGR", "width", G_TYPE_INT, self->width_,
            "height", G_TYPE_INT, self->height_, "framerate", GST_TYPE_FRACTION, self->fps_, 1,
            nullptr);
        g_object_set(appsrc, "caps", caps, "is-live", TRUE, "format", GST_FORMAT_TIME, nullptr);
        gst_caps_unref(caps);

        gst_object_unref(appsrc);
    }

    void start_rtsp_server() {
        gst_thread_ = std::thread([this]() {
            auto* server = gst_rtsp_server_new();
            gst_rtsp_server_set_service(server, std::to_string(port_).c_str());

            auto* mounts  = gst_rtsp_server_get_mount_points(server);
            auto* factory = gst_rtsp_media_factory_new();

            std::string launch_pipeline = "( appsrc name=mysrc is-live=true format=time ! "
                                          "videoconvert ! x264enc tune=zerolatency bitrate="
                                        + std::to_string(bitrate_)
                                        + " speed-preset=ultrafast ! rtph264pay name=pay0 pt=96 )";

            gst_rtsp_media_factory_set_launch(factory, launch_pipeline.c_str());
            g_signal_connect(factory, "media-configure", G_CALLBACK(media_configure), this);
            gst_rtsp_media_factory_set_shared(factory, TRUE);
            gst_rtsp_mount_points_add_factory(mounts, mount_point_.c_str(), factory);
            gst_object_unref(mounts);

            gst_rtsp_server_attach(server, nullptr);

            RCLCPP_INFO(
                get_logger(), "RTSP server at rtsp://0.0.0.0:%d%s", port_, mount_point_.c_str());

            loop_ = g_main_loop_new(nullptr, FALSE);
            g_main_loop_run(loop_);

            gst_object_unref(server);
        });
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;

    cv::Mat current_frame_;
    std::mutex frame_mutex_;
    GstClockTime timestamp_ = 0;

    GMainLoop* loop_ = nullptr;
    std::thread gst_thread_;

    int port_                = 8554;
    int bitrate_             = 2000;
    int width_               = 640;
    int height_              = 480;
    int fps_                 = 30;
    std::string mount_point_ = "/cam";
};

} // namespace dogbot_core::camera

RCLCPP_COMPONENTS_REGISTER_NODE(dogbot_core::camera::RtspStreamNode)
