// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2018 Intel Corporation. All Rights Reserved.

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This set of tests is valid for any number and combination of RealSense cameras, including R200 and F200 //
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <cmath>
#include "unit-tests-common.h"
#include "../include/librealsense2/rs_advanced_mode.hpp"
#include <librealsense2/hpp/rs_frame.hpp>
#include <iostream>
#include <chrono>
#include <ctime>
#include <algorithm>
#include <librealsense2/rsutil.h>

# define SECTION_FROM_TEST_NAME space_to_underscore(Catch::getCurrentContext().getResultCapture()->getCurrentTestName()).c_str()

typedef struct _sw_context
{
    rs2::software_device sdev;
    std::map<std::string, rs2::software_sensor> sw_sensors;
    std::map<std::string, rs2::syncer> sw_syncers;
    std::map<std::string, std::map<rs2_stream, rs2::stream_profile>> sw_stream_profiles;
} sw_context;

rs2_software_video_frame create_sw_frame(const rs2::video_frame& f, rs2::stream_profile profile)
{
    rs2_software_video_frame new_frame = {
        (void*)f.get_data(),
        [](void*) {},
        f.get_width() * f.get_bytes_per_pixel(),
        f.get_bytes_per_pixel(),
        f.get_timestamp(),
        f.get_frame_timestamp_domain(),
        static_cast<int>(f.get_frame_number()),
        profile
    };
    return new_frame;
}

class processing_record_block
{
public:
    virtual void record(std::string sensor_name, const rs2::frame& frame, sw_context sctx) = 0;
    virtual rs2::frame process(const rs2::frame& frame) = 0;
};

class align_record_block : public processing_record_block
{
public:
    align_record_block(rs2_stream align_to, rs2_stream align_from) : _align(align_to), _align_from(align_from) {}
    virtual void record(std::string sensor_name, const rs2::frame& frame, sw_context sctx) override
    {
        auto ss = sctx.sw_sensors[sensor_name];
        ss.on_video_frame(create_sw_frame(frame, sctx.sw_stream_profiles[sensor_name][_align_from]));
        ss.stop();
        ss.close();
    }
    virtual rs2::frame process(const rs2::frame& frame) override
    {
        auto fs = frame.as<rs2::frameset>();
        return _align.process(fs).first_or_default(_align_from);
    }

private:
    rs2_stream _align_from;
    rs2::align _align;
};

class pointcloud_record_block : public processing_record_block
{
public:
    pointcloud_record_block() {}
    virtual void record(std::string sensor_name, const rs2::frame& frame, sw_context sctx) override
    {
        auto profile = sctx.sw_stream_profiles[sensor_name][RS2_STREAM_DEPTH].as<rs2::video_stream_profile>();
        rs2_software_video_frame points_frame = {
            (void*)frame.get_data(),
            [](void*) {},
            profile.width() * 20,
            20,
            frame.get_timestamp(),
            frame.get_frame_timestamp_domain(),
            static_cast<int>(frame.get_frame_number()),
            sctx.sw_stream_profiles[sensor_name][RS2_STREAM_DEPTH]
        };

        auto ss = sctx.sw_sensors[sensor_name];
        ss.on_video_frame(points_frame);
        ss.stop();
        ss.close();
    }
    virtual rs2::frame process(const rs2::frame& frame) override
    {
        auto fs = frame.as<rs2::frameset>();
        return _pc.calculate(fs.get_depth_frame());
    }

private:
    rs2::pointcloud _pc;
};

std::string get_sensor_name(rs2::video_stream_profile c, rs2::video_stream_profile d)
{
    std::string dres = std::to_string(d.width()) + "x" + std::to_string(d.height());
    std::string cres = std::to_string(c.width()) + "x" + std::to_string(c.height());
    std::string name = "depth_" + dres + "_color_" + std::to_string(c.format()) + "_" + cres;
    return name;
}

rs2::stream_profile init_stream_profile(rs2::software_sensor ss, rs2::video_stream_profile stream_profile)
{
    rs2_video_stream new_stream = {
        stream_profile.stream_type(),
        stream_profile.stream_index(),
        stream_profile.unique_id(),
        stream_profile.width(),
        stream_profile.height(),
        stream_profile.fps(),
        stream_profile.height(),
        stream_profile.format(),
        stream_profile.get_intrinsics()
    };

    return ss.add_video_stream(new_stream);
}

std::vector<rs2::stream_profile> init_stream_profiles(sw_context& sctx, rs2::software_sensor ss, std::string sensor_name, rs2::video_stream_profile c, rs2::video_stream_profile d)
{
    sctx.sw_stream_profiles[sensor_name][RS2_STREAM_DEPTH] = init_stream_profile(ss, d);
    sctx.sw_stream_profiles[sensor_name][RS2_STREAM_COLOR] = init_stream_profile(ss, c);
    std::vector<rs2::stream_profile> profiles = {
        sctx.sw_stream_profiles[sensor_name][RS2_STREAM_DEPTH],
        sctx.sw_stream_profiles[sensor_name][RS2_STREAM_COLOR]
    };
    return profiles;
}

sw_context init_sw_device(std::vector<rs2::video_stream_profile> depth_profiles,
    std::vector<rs2::video_stream_profile> color_profiles, float depth_units)
{
    sw_context sctx;
    for (auto depth_profile : depth_profiles)
    {
        for (auto color_profile : color_profiles)
        {
            if (depth_profile.width() == color_profile.width() && depth_profile.height() == color_profile.height())
                continue;

            std::string name = get_sensor_name(color_profile, depth_profile);
            auto sensor = sctx.sdev.add_sensor(name);

            sensor.add_read_only_option(RS2_OPTION_DEPTH_UNITS, depth_units);

            sensor.open(init_stream_profiles(sctx, sensor, name, color_profile, depth_profile));
            rs2::syncer sync;
            sensor.start(sync);

            sctx.sw_sensors[name] = sensor;
            sctx.sw_syncers[name] = sync;
            break;
        }
    }
    return sctx;
}

void record_sw_frame(std::string sensor_name, rs2::frameset fs, sw_context sctx)
{
    auto ss = sctx.sw_sensors[sensor_name];
    ss.on_video_frame(create_sw_frame(fs.get_depth_frame(), fs.get_depth_frame().get_profile().as<rs2::video_stream_profile>()));
    ss.on_video_frame(create_sw_frame(fs.get_color_frame(), fs.get_color_frame().get_profile().as<rs2::video_stream_profile>()));

    ss.stop();
    ss.close();
}

TEST_CASE("Record software-device all resolutions", "[software-device][record]")
{
    rs2::context ctx;
    if (!make_context(SECTION_FROM_TEST_NAME, &ctx))
        return;

    auto dev = ctx.query_devices()[0];
    auto sensors = dev.query_sensors();

    std::vector<rs2::video_stream_profile> depth_profiles;
    for (auto p : sensors[0].get_stream_profiles())
    {
        if (p.stream_type() == rs2_stream::RS2_STREAM_DEPTH)
        {
            auto pv = p.as<rs2::video_stream_profile>();
            if (!std::any_of(depth_profiles.begin(), depth_profiles.end(), [&](rs2::video_stream_profile i)
            {
                return i.height() == pv.height() && i.width() == pv.width() && i.format() == pv.format();
            }))
            {
                depth_profiles.push_back(pv);
            }
        }
    }

    std::vector<rs2::video_stream_profile> color_profiles;
    for (auto p : sensors[1].get_stream_profiles())
    {
        if (p.stream_type() == rs2_stream::RS2_STREAM_COLOR)
        {
            auto pv = p.as<rs2::video_stream_profile>();
            if (!std::any_of(color_profiles.begin(), color_profiles.end(), [&](rs2::video_stream_profile i)
            {
                return i.height() == pv.height() && i.width() == pv.width() && i.format() == pv.format();
            }))
            {
                color_profiles.push_back(pv);
                break;
            }
        }
    }

    auto sctx = init_sw_device(depth_profiles, color_profiles, sensors[0].get_option(RS2_OPTION_DEPTH_UNITS));

    rs2::recorder recorder("all_combinations_depth_color.bag", sctx.sdev);

    for (auto depth_profile : depth_profiles)
    {
        for (auto color_profile : color_profiles)
        {
            if (depth_profile.width() == color_profile.width() && depth_profile.height() == color_profile.height())
                continue;
            rs2::syncer sync;
            sensors[0].open(depth_profile);
            sensors[1].open(color_profile);

            sensors[0].start(sync);
            sensors[1].start(sync);

            while (true)
            {
                auto fs = sync.wait_for_frames();
                if (fs.size() == 2)
                {
                    std::cout << "Recording : " << "Depth : " << depth_profile.width() << "x" << depth_profile.height() << ", Color : " << color_profile.width() << "x" << color_profile.height() << std::endl;
                    std::string sensor_name = get_sensor_name(color_profile, depth_profile);
                    record_sw_frame(sensor_name, fs, sctx);
                    break;
                }
            }
            sensors[0].stop();
            sensors[1].stop();
            sensors[0].close();
            sensors[1].close();
        }
    }
}

std::vector<rs2::frameset> get_composite_frames(std::vector<rs2::sensor> sensors)
{
    std::vector<rs2::frameset> composite_frames;

    std::vector<rs2::frame> frames;
    std::mutex frame_processor_lock;
    rs2::processing_block frame_processor([&](rs2::frame data, rs2::frame_source& source)
    {
        std::lock_guard<std::mutex> lock(frame_processor_lock);
        frames.push_back(data);
        if (frames.size() == 2)
        {
            source.frame_ready(source.allocate_composite_frame(frames));
            frames.clear();
        }
    });

    rs2::frame_queue postprocessed_frames;
    frame_processor >> postprocessed_frames;

    for (auto s : sensors)
    {
        s.open(s.get_stream_profiles());
        s.start([&](rs2::frame f)
        {
            frame_processor.invoke(f);
        });
    }

    while (composite_frames.size() < sensors.size())
    {
        rs2::frameset composite_fs;
        if (postprocessed_frames.try_wait_for_frame(&composite_fs))
        {
            composite_fs.keep();
            composite_frames.push_back(composite_fs);
        }
    }

    return composite_frames;
}

std::vector<rs2::frame> get_frames(std::vector<rs2::sensor> sensors)
{
    std::vector<rs2::frame> frames;
    std::mutex frames_lock;

    for (auto s : sensors)
    {
        s.open(s.get_stream_profiles());
        s.start([&](rs2::frame f)
        {
            std::lock_guard<std::mutex> lock(frames_lock);
            frames.push_back(f);
        });
    }

    while (frames.size() < sensors.size())
    {
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }

    return frames;
}

sw_context init_sw_device(std::vector<std::string> sensor_names, std::vector<rs2::frame> processed_frames)
{
    sw_context sctx;
    for (int i = 0; i < processed_frames.size(); i++)
    {
        auto processed_frame = processed_frames[i];

        auto sensor = sctx.sdev.add_sensor(sensor_names[i]);

        auto profile = processed_frame.get_profile().as<rs2::video_stream_profile>();

        sctx.sw_stream_profiles[sensor_names[i]][profile.stream_type()] = init_stream_profile(sensor, profile);

        //register_extrinsics_from_frame(processed_frame, sctx, name);

        sensor.open(sctx.sw_stream_profiles[sensor_names[i]][profile.stream_type()]);
        rs2::syncer sync;
        sensor.start(sync);

        sctx.sw_sensors[sensor_names[i]] = sensor;
        sctx.sw_syncers[sensor_names[i]] = sync;
    }
    return sctx;
}

void record_frames_all_res(processing_record_block& record_block, std::string file)
{
    rs2::context ctx;
    if (!make_context(SECTION_FROM_TEST_NAME, &ctx))
        return;

    auto dev = ctx.load_device("all_combinations_depth_color.bag");
    dev.set_real_time(false);

    std::cout << "Recording was loaded" << std::endl;

    std::vector<rs2::sensor> sensors = dev.query_sensors();
    auto original_frames = get_composite_frames(sensors);

    std::cout << "Recieved all recorded composite frames" << std::endl;

    std::vector<rs2::frame> processed_frames;
    std::vector<std::string> sensor_names;

    for (auto f : original_frames)
    {
        auto processed_frame = record_block.process(f);
        processed_frame.keep();
        processed_frames.push_back(processed_frame);

        auto color_stream_profile = f.get_color_frame().get_profile().as<rs2::video_stream_profile>();
        auto depth_stream_profile = f.get_depth_frame().get_profile().as<rs2::video_stream_profile>();
        sensor_names.push_back(get_sensor_name(color_stream_profile, depth_stream_profile));
    }
    std::cout << "All frames were processed" << std::endl;

    auto sctx = init_sw_device(sensor_names, processed_frames);
    rs2::recorder recorder(file, sctx.sdev);

    std::cout << "SW device initilized" << std::endl;

    for (int i = 0; i < processed_frames.size(); i++)
    {
        record_block.record(sensor_names[i], processed_frames[i], sctx);
    }
    std::cout << "All frames were recorded" << std::endl;

    for (auto s : sensors)
    {
        s.stop();
        s.close();
    }
    std::cout << "Done" << std::endl;
}

TEST_CASE("Record align color to depth software-device all resolutions", "[software-device][record][align]")
{
    record_frames_all_res(align_record_block(RS2_STREAM_DEPTH, RS2_STREAM_COLOR), "[aligned_2d]_all_combinations_depth_color.bag");
}

TEST_CASE("Record align depth to color software-device all resolutions", "[software-device][record][align]")
{
    record_frames_all_res(align_record_block(RS2_STREAM_COLOR, RS2_STREAM_DEPTH), "[aligned_2c]_all_combinations_depth_color.bag");
}

void validate_ppf_results(const rs2::frame& result_frame, const rs2::frame& reference_frame)
{
    auto result_profile = result_frame.get_profile().as<rs2::video_stream_profile>();
    REQUIRE(result_profile);
    CAPTURE(result_profile.width());
    CAPTURE(result_profile.height());

    auto reference_profile = reference_frame.get_profile().as<rs2::video_stream_profile>();
    REQUIRE(reference_profile);
    CAPTURE(reference_profile.width());
    CAPTURE(reference_profile.height());

    REQUIRE(result_profile.width() == reference_profile.width());
    REQUIRE(result_profile.height() == reference_profile.height());

    auto pixels_as_bytes = reference_frame.as<rs2::video_frame>().get_bytes_per_pixel() * result_profile.width() * result_profile.height();

    // Pixel-by-pixel comparison of the resulted filtered depth vs data ercorded with external tool
    auto v1 = reinterpret_cast<const uint8_t*>(result_frame.get_data());
    auto v2 = reinterpret_cast<const uint8_t*>(reference_frame.get_data());

    REQUIRE(std::memcmp(v1, v2, pixels_as_bytes) == 0);
}

void compare_processed_frames_vs_recorded_frames(processing_record_block& record_block, std::string file)
{
    rs2::context ctx;
    if (!make_context(SECTION_FROM_TEST_NAME, &ctx))
        return;

    auto dev = ctx.load_device("all_combinations_depth_color.bag");
    dev.set_real_time(false);

    std::vector<rs2::sensor> sensors = dev.query_sensors();
    auto frames = get_composite_frames(sensors);

    //std::string folder_name = get_folder_path(special_folder::temp_folder);

    auto ref_dev = ctx.load_device(file);
    ref_dev.set_real_time(false);

    std::vector<rs2::sensor> ref_sensors = ref_dev.query_sensors();
    auto ref_frames = get_frames(ref_sensors);



    std::cout << "---------------------------------------------------------------------------------------------\n";
    std::cout << "Calculated time interval to align a frame\n";
    std::cout << "---------------------------------------------------------------------------------------------\n";
    for (int i = 0; i < frames.size(); i++)
    {
        auto d = frames[i].get_depth_frame().get_profile().as<rs2::video_stream_profile>();
        auto c = frames[i].get_color_frame().get_profile().as<rs2::video_stream_profile>();
        auto started = std::chrono::high_resolution_clock::now();
        auto fs_res = record_block.process(frames[i]);
        auto done = std::chrono::high_resolution_clock::now();

        std::cout << "DEPTH " << std::setw(4) << d.format() << " " << std::setw(10) << std::to_string(d.width()) + "x" + std::to_string(d.height()) << " | " <<
            "COLOR " << std::setw(6) << c.format() << " " << std::setw(10) << std::to_string(c.width()) + "x" + std::to_string(c.height()) << std::setw(4) << " [" <<
            std::setw(6) << std::chrono::duration_cast<std::chrono::microseconds>(done - started).count() << " us]" << std::endl;

        validate_ppf_results(fs_res, ref_frames[i]);
    }

    for (auto s : sensors)
    {
        s.stop();
        s.close();
    }

    for (auto s : ref_sensors)
    {
        s.stop();
        s.close();
    }
}

TEST_CASE("Test align color to depth from recording", "[software-device][align]")
{
    compare_processed_frames_vs_recorded_frames(align_record_block(RS2_STREAM_DEPTH, RS2_STREAM_COLOR), "[aligned_2d]_all_combinations_depth_color.bag");
}

TEST_CASE("Test align depth to color from recording", "[software-device][align]")
{
    compare_processed_frames_vs_recorded_frames(align_record_block(RS2_STREAM_COLOR, RS2_STREAM_DEPTH), "[aligned_2c]_all_combinations_depth_color.bag");
}

TEST_CASE("Record point cloud software-device all resolutions", "[software-device][record][point-cloud]")
{
    record_frames_all_res(pointcloud_record_block(), "[pointcloud]_all_combinations_depth_color.bag");
}

TEST_CASE("Test point cloud from recording", "[software-device][point-cloud]")
{
    compare_processed_frames_vs_recorded_frames(pointcloud_record_block(), "[pointcloud]_all_combinations_depth_color.bag");
}