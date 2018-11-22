// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2018 Intel Corporation. All Rights Reserved.

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This set of tests is valid for any number and combination of RealSense cameras, including R200 and F200 //
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include "unit-tests-common.h"
#include "unit-tests-post-processing.h"
#include "../include/librealsense2/rs_advanced_mode.hpp"
#include <librealsense2/hpp/rs_frame.hpp>
#include <cmath>
#include <iostream>
#include <chrono>
#include <ctime>
#include <algorithm>


# define SECTION_FROM_TEST_NAME space_to_underscore(Catch::getCurrentContext().getResultCapture()->getCurrentTestName()).c_str()

class post_processing_filters
{
public:
    post_processing_filters(void) : depth_to_disparity(true),disparity_to_depth(false) {};
    ~post_processing_filters() noexcept {};

    void configure(const ppf_test_config& filters_cfg);
    rs2::frame process(rs2::frame input_frame);

private:
    post_processing_filters(const post_processing_filters& other);
    post_processing_filters(post_processing_filters&& other);

    // Declare filters
    rs2::decimation_filter  dec_filter;     // Decimation - frame downsampling using median filter
    rs2::spatial_filter     spat_filter;    // Spatial    - edge-preserving spatial smoothing
    rs2::temporal_filter    temp_filter;    // Temporal   - reduces temporal noise
    rs2::hole_filling_filter hole_filling_filter; // try reconstruct the missing data

    // Declare disparity transform from depth to disparity and vice versa
    rs2::disparity_transform depth_to_disparity;
    rs2::disparity_transform disparity_to_depth;

    bool dec_pb = false;
    bool spat_pb = false;
    bool temp_pb = false;
    bool holes_pb = false;

};

void post_processing_filters::configure(const ppf_test_config& filters_cfg)
{
    // Reconfigure the post-processing according to the test spec
    dec_pb = (filters_cfg.downsample_scale != 1);
    dec_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, (float)filters_cfg.downsample_scale);

    if (spat_pb = filters_cfg.spatial_filter)
    {
        spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, filters_cfg.spatial_alpha);
        spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, filters_cfg.spatial_delta);
        spat_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, (float)filters_cfg.spatial_iterations);
        //spat_filter.set_option(RS2_OPTION_HOLES_FILL, filters_cfg.holes_filling_mode);      // Currently disabled
    }

    if (temp_pb = filters_cfg.temporal_filter)
    {
        temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, filters_cfg.temporal_alpha);
        temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, filters_cfg.temporal_delta);
        temp_filter.set_option(RS2_OPTION_HOLES_FILL, filters_cfg.temporal_persistence);
    }

    if (holes_pb = filters_cfg.holes_filter)
    {
        hole_filling_filter.set_option(RS2_OPTION_HOLES_FILL, filters_cfg.holes_filling_mode);
    }
}

rs2::frame post_processing_filters::process(rs2::frame input)
{
    auto processed = input;

    // The filters are applied in the order mandated by reference design to enable byte-by-byte results verification
    // Decimation -> Depth2Disparity -> Spatial ->Temporal -> Disparity2Depth -> HolesFilling

    if (dec_pb)
        processed = dec_filter.process(processed);

    // Domain transform is mandatory according to the reference design
    processed = depth_to_disparity.process(processed);

    if (spat_pb)
        processed = spat_filter.process(processed);

    if (temp_pb)
        processed = temp_filter.process(processed);

    processed= disparity_to_depth.process(processed);

    if (holes_pb)
        processed = hole_filling_filter.process(processed);

    return processed;
}

bool validate_ppf_results(rs2::frame result_depth, const ppf_test_config& reference_data, size_t frame_idx)
{
    std::vector<uint16_t> diff2orig;
    std::vector<uint16_t> diff2ref;

    // Basic sanity scenario with no filters applied.
    // validating domain transform in/out conversion. Requiring input=output
    bool domain_transform_only = (reference_data.downsample_scale == 1) &&
        (!reference_data.spatial_filter) && (!reference_data.temporal_filter);

    auto result_profile = result_depth.get_profile().as<rs2::video_stream_profile>();
    REQUIRE(result_profile);
    CAPTURE(result_profile.width());
    CAPTURE(result_profile.height());

    REQUIRE(result_profile.width() == reference_data.output_res_x);
    REQUIRE(result_profile.height() == reference_data.output_res_y);

    auto pixels = result_profile.width()*result_profile.height();
    diff2ref.resize(pixels);
    if (domain_transform_only)
        diff2orig.resize(pixels);

    // Pixel-by-pixel comparison of the resulted filtered depth vs data ercorded with external tool
    auto v1 = reinterpret_cast<const uint16_t*>(result_depth.get_data());
    auto v2 = reinterpret_cast<const uint16_t*>(reference_data._output_frames[frame_idx].data());

    for (auto i = 0; i < pixels; i++)
    {
        uint16_t diff = std::abs(*v1++ - *v2++);
        diff2ref[i] = diff;
    }

    // validating depth<->disparity domain transformation is lostless.
    if (domain_transform_only)
        REQUIRE(profile_diffs("./DomainTransform.txt",diff2orig, 0, 0, frame_idx));

    // Validate the filters
    // The differences between the reference code and librealsense implementation are byte-compared below
    return profile_diffs("./Filterstransform.txt", diff2ref, 0.f, 0, frame_idx);
}

void compare_frame_md(rs2::frame origin_depth, rs2::frame result_depth)
{
    for (auto i = 0; i < rs2_frame_metadata_value::RS2_FRAME_METADATA_COUNT; i++)
    {
        bool origin_supported = origin_depth.supports_frame_metadata((rs2_frame_metadata_value)i);
        bool result_supported = result_depth.supports_frame_metadata((rs2_frame_metadata_value)i);
        REQUIRE(origin_supported == result_supported);
        if (origin_supported && result_supported)
        {
            //FRAME_TIMESTAMP and SENSOR_TIMESTAMP metadatas are not included in post proccesing frames,
            //TIME_OF_ARRIVAL continues to increase  after post proccesing
            if (i == RS2_FRAME_METADATA_FRAME_TIMESTAMP ||
                i == RS2_FRAME_METADATA_SENSOR_TIMESTAMP ||
                i == RS2_FRAME_METADATA_TIME_OF_ARRIVAL) continue;
            rs2_metadata_type origin_val = origin_depth.get_frame_metadata((rs2_frame_metadata_value)i);
            rs2_metadata_type result_val = result_depth.get_frame_metadata((rs2_frame_metadata_value)i);
            REQUIRE(origin_val == result_val);
        }
    }
}

// Test file name  , Filters configuraiton
const std::vector< std::pair<std::string, std::string>> ppf_test_cases = {
    // All the tests below include depth-disparity domain transformation
    // Downsample scales 2/3
    { "1525186403504",  "D415_DS(2)" },
{ "1525186407536",  "D415_DS(3)" },
// Downsample + Hole-Filling modes 0/1/2
{ "1525072818314",  "D415_DS(1)_HoleFill(0)" },
{ "1525072823227",  "D415_DS(1)_HoleFill(1)" },
{ "1524668713358",  "D435_DS(3)_HoleFill(2)" },
// Downsample + Spatial Filter parameters
{ "1525267760676",  "D415_DS(2)+Spat(A:0.85/D:32/I:3)" },
// Downsample + Temporal Filter
{ "1525266028697",  "D415_DS(2)+Temp(A:0.25/D:15/P:1)" },
{ "1525265554250",  "D415_DS(2)+Temp(A:0.25/D:15/P:3)" },
{ "1525266069476",  "D415_DS(2)+Temp(A:0.25/D:15/P:5)" },
{ "1525266120520",  "D415_DS(3)+Temp(A:0.25/D:15/P:7)" },
// Downsample + Spatial + Temporal (+ Hole-Filling)
{ "1525267168585",  "D415_DS(2)_Spat(A:0.85/D:32/I:3)_Temp(A:0.25/D:15/P:0)" },
{ "1525089539880",  "D415_DS(2)_Spat(A:0.85/D:32/I:3)_Temp(A:0.25/D:15/P:0)_HoleFill(1)" },
};

// The test is intended to check the results of filters applied on a sequence of frames, specifically the temporal filter
// that preserves an internal state. The test utilizes rosbag recordings
TEST_CASE("Post-Processing Filters sequence validation", "[software-device][post-processing-filters]")
{
    rs2::context ctx;

    if (make_context(SECTION_FROM_TEST_NAME, &ctx))
    {
        ppf_test_config test_cfg;

        for (auto& ppf_test : ppf_test_cases)
        {
            CAPTURE(ppf_test.first);
            CAPTURE(ppf_test.second);

            WARN("PPF test " << ppf_test.first << "[" << ppf_test.second << "]");

            // Load the data from configuration and raw frame files
            if (!load_test_configuration(ppf_test.first, test_cfg))
                continue;

            post_processing_filters ppf;

            // Apply the retrieved configuration onto a local post-processing chain of filters
            REQUIRE_NOTHROW(ppf.configure(test_cfg));

            rs2::software_device dev; // Create software-only device
            auto depth_sensor = dev.add_sensor("Depth");

            int width = test_cfg.input_res_x;
            int height = test_cfg.input_res_y;
            int depth_bpp = 2; //16bit unsigned
            int frame_number = 1;
            rs2_intrinsics depth_intrinsics = { width, height,
                width / 2.f, height / 2.f,                      // Principal point (N/A in this test)
                test_cfg.focal_length ,test_cfg.focal_length,   // Focal Length
                RS2_DISTORTION_BROWN_CONRADY ,{ 0,0,0,0,0 } };

            auto depth_stream_profile = depth_sensor.add_video_stream({ RS2_STREAM_DEPTH, 0, 0, width, height, 30, depth_bpp, RS2_FORMAT_Z16, depth_intrinsics });
            depth_sensor.add_read_only_option(RS2_OPTION_DEPTH_UNITS, test_cfg.depth_units);
            depth_sensor.add_read_only_option(RS2_OPTION_STEREO_BASELINE, test_cfg.stereo_baseline);

            // Establish the required chain of filters
            dev.create_matcher(RS2_MATCHER_DLR_C);
            rs2::syncer sync;

            depth_sensor.open(depth_stream_profile);
            depth_sensor.start(sync);

            size_t frames = (test_cfg.frames_sequence_size > 1) ? test_cfg.frames_sequence_size : 1;
            for (auto i = 0; i < frames; i++)
            {
                // Inject input frame
                depth_sensor.on_video_frame({ test_cfg._input_frames[i].data(), // Frame pixels from capture API
                    [](void*) {},                   // Custom deleter (if required)
                    (int)test_cfg.input_res_x *depth_bpp,    // Stride
                    depth_bpp,                          // Bytes-per-pixels
                    (rs2_time_t)frame_number + i,      // Timestamp
                    RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME,   // Clock Domain
                    frame_number,                       // Frame# for potential sync services
                    depth_stream_profile });            // Depth stream profile

                rs2::frameset fset = sync.wait_for_frames();
                REQUIRE(fset);
                rs2::frame depth = fset.first_or_default(RS2_STREAM_DEPTH);
                REQUIRE(depth);

                // ... here the actual filters are being applied
                auto filtered_depth = ppf.process(depth);

                // Compare the resulted frame versus input
                validate_ppf_results(filtered_depth, test_cfg, i);
            }
        }
    }
}

TEST_CASE("Post-Processing Filters metadata validation", "[software-device][post-processing-filters]")
{
    rs2::context ctx;

    if (make_context(SECTION_FROM_TEST_NAME, &ctx))
    {
        ppf_test_config test_cfg;
        for (auto& ppf_test : ppf_test_cases)
        {
            CAPTURE(ppf_test.first);
            CAPTURE(ppf_test.second);

            WARN("PPF test " << ppf_test.first << "[" << ppf_test.second << "]");

            // Load the data from configuration and raw frame files
            if (!load_test_configuration(ppf_test.first, test_cfg))
                continue;

            post_processing_filters ppf;

            // Apply the retrieved configuration onto a local post-processing chain of filters
            REQUIRE_NOTHROW(ppf.configure(test_cfg));

            rs2::software_device dev; // Create software-only device
            auto depth_sensor = dev.add_sensor("Depth");

            int width = test_cfg.input_res_x;
            int height = test_cfg.input_res_y;
            int depth_bpp = 2; //16bit unsigned
            int frame_number = 1;
            rs2_intrinsics depth_intrinsics = { width, height,
                width / 2.f, height / 2.f,                      // Principal point (N/A in this test)
                test_cfg.focal_length ,test_cfg.focal_length,   // Focal Length
                RS2_DISTORTION_BROWN_CONRADY ,{ 0,0,0,0,0 } };

            auto depth_stream_profile = depth_sensor.add_video_stream({ RS2_STREAM_DEPTH, 0, 0, width, height, 30, depth_bpp, RS2_FORMAT_Z16, depth_intrinsics });

            // Establish the required chain of filters
            dev.create_matcher(RS2_MATCHER_DLR_C);
            rs2::syncer sync;

            depth_sensor.open(depth_stream_profile);
            depth_sensor.start(sync);

            size_t frames = (test_cfg.frames_sequence_size > 1) ? test_cfg.frames_sequence_size : 1;
            for (auto i = 0; i < frames; i++)
            {
                //set next frames metadata
                for (auto i = 0; i < rs2_frame_metadata_value::RS2_FRAME_METADATA_COUNT; i++)
                    depth_sensor.set_metadata((rs2_frame_metadata_value)i, rand());

                // Inject input frame
                depth_sensor.on_video_frame({ test_cfg._input_frames[i].data(), // Frame pixels from capture API
                    [](void*) {},                   // Custom deleter (if required)
                    (int)test_cfg.input_res_x *depth_bpp,    // Stride
                    depth_bpp,                          // Bytes-per-pixels
                    (rs2_time_t)frame_number + i,      // Timestamp
                    RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME,   // Clock Domain
                    frame_number,                       // Frame# for potential sync services
                    depth_stream_profile });            // Depth stream profile

                rs2::frameset fset = sync.wait_for_frames();
                REQUIRE(fset);
                rs2::frame depth = fset.first_or_default(RS2_STREAM_DEPTH);
                REQUIRE(depth);

                // ... here the actual filters are being applied
                auto filtered_depth = ppf.process(depth);

                // Compare the resulted frame metadata versus input
                compare_frame_md(depth, filtered_depth);
            }
        }
    }
}

bool is_subset(rs2::frameset full, rs2::frameset sub)
{
    if (!sub.is<rs2::frameset>())
        return false;
    if (full.size() == 0 && sub.size() == 0)
        return false;
    for (auto f : full)
    {
        if (!sub.first(f.get_profile().stream_type(), f.get_profile().format()))
            return false;
    }
    return true;
}

bool is_equal(rs2::frameset org, rs2::frameset processed)
{
    if (!org.is<rs2::frameset>() || !processed.is<rs2::frameset>())
        return false;
    if (org.size() != processed.size() || org.size() == 0)
        return false;
    for (auto o : org)
    {
        auto curr_profile = o.get_profile();
        bool found = false;
        processed.foreach([&curr_profile, &found](const rs2::frame& f)
        {
            auto processed_profile = f.get_profile();
            if (curr_profile.unique_id() == processed_profile.unique_id())
                found = true;
        });
        if(!found)
            return false;
    }
    return true;
}

TEST_CASE("Post-Processing expected output", "[post-processing-filters]")
{
    rs2::context ctx;

    if (!make_context(SECTION_FROM_TEST_NAME, &ctx))
        return;

    rs2::temporal_filter temporal;
    rs2::hole_filling_filter hole_filling;
    rs2::spatial_filter spatial;
    rs2::decimation_filter decimation(4);
    rs2::align aligner(RS2_STREAM_COLOR);
    rs2::colorizer depth_colorizer;
    rs2::disparity_transform to_disp;
    rs2::disparity_transform from_disp(false);

    rs2::config cfg;
    cfg.enable_all_streams();

    rs2::pipeline pipe(ctx);
    auto profile = pipe.start(cfg);

    bool supports_disparity = false;
    for (auto s : profile.get_device().query_sensors())
    {
        if (s.supports(RS2_OPTION_STEREO_BASELINE))
        {
            supports_disparity = true;
            break;
        }
    }

    rs2::frameset original = pipe.wait_for_frames();

    //set to set
    rs2::frameset temp_processed_set = original.apply_filter(temporal);
    REQUIRE(is_subset(original, temp_processed_set));
    REQUIRE(is_subset(temp_processed_set, original));

    rs2::frameset hole_processed_set = original.apply_filter(hole_filling);
    REQUIRE(is_subset(original, hole_processed_set));
    REQUIRE(is_subset(hole_processed_set, original));

    rs2::frameset spatial_processed_set = original.apply_filter(spatial);
    REQUIRE(is_subset(original, spatial_processed_set));
    REQUIRE(is_subset(spatial_processed_set, original));

    rs2::frameset decimation_processed_set = original.apply_filter(decimation);
    REQUIRE(is_subset(original, decimation_processed_set));
    REQUIRE(is_subset(decimation_processed_set, original));

    rs2::frameset align_processed_set = original.apply_filter(aligner);
    REQUIRE(is_subset(original, align_processed_set));
    REQUIRE(is_subset(align_processed_set, original));

    rs2::frameset colorizer_processed_set = original.apply_filter(depth_colorizer);
    REQUIRE(is_subset(original, colorizer_processed_set));
    REQUIRE_THROWS(is_subset(colorizer_processed_set, original));

    rs2::frameset to_disp_processed_set = original.apply_filter(to_disp);
    if(supports_disparity)
        REQUIRE_THROWS(is_subset(to_disp_processed_set, original));

    rs2::frameset from_disp_processed_set = original.apply_filter(from_disp);//should bypass
    REQUIRE(is_equal(original, from_disp_processed_set));

    //single to single
    rs2::video_frame org_depth = original.get_depth_frame();

    rs2::video_frame temp_processed_frame = org_depth.apply_filter(temporal);
    REQUIRE_FALSE(temp_processed_frame.is<rs2::frameset>());
    REQUIRE(temp_processed_frame.get_profile().stream_type() == RS2_STREAM_DEPTH);
    REQUIRE(temp_processed_frame.get_profile().format() == RS2_FORMAT_Z16);
    REQUIRE(org_depth.get_width() == temp_processed_frame.get_width());

    rs2::video_frame hole_processed_frame = org_depth.apply_filter(hole_filling);
    REQUIRE_FALSE(hole_processed_frame.is<rs2::frameset>());
    REQUIRE(hole_processed_frame.get_profile().stream_type() == RS2_STREAM_DEPTH);
    REQUIRE(hole_processed_frame.get_profile().format() == RS2_FORMAT_Z16);
    REQUIRE(org_depth.get_width() == hole_processed_frame.get_width());

    rs2::video_frame spatial_processed_frame = org_depth.apply_filter(spatial);
    REQUIRE_FALSE(spatial_processed_frame.is<rs2::frameset>());
    REQUIRE(spatial_processed_frame.get_profile().stream_type() == RS2_STREAM_DEPTH);
    REQUIRE(spatial_processed_frame.get_profile().format() == RS2_FORMAT_Z16);
    REQUIRE(org_depth.get_width() == spatial_processed_frame.get_width());

    rs2::video_frame decimation_processed_frame = org_depth.apply_filter(decimation);
    REQUIRE_FALSE(decimation_processed_frame.is<rs2::frameset>());
    REQUIRE(decimation_processed_frame.get_profile().stream_type() == RS2_STREAM_DEPTH);
    REQUIRE(decimation_processed_frame.get_profile().format() == RS2_FORMAT_Z16);
    REQUIRE(org_depth.get_width() > decimation_processed_frame.get_width());

    rs2::video_frame colorizer_processed_frame = org_depth.apply_filter(depth_colorizer);
    REQUIRE_FALSE(colorizer_processed_frame.is<rs2::frameset>());
    REQUIRE(colorizer_processed_frame.get_profile().stream_type() == RS2_STREAM_DEPTH);
    REQUIRE(colorizer_processed_frame.get_profile().format() == RS2_FORMAT_RGB8);
    REQUIRE(org_depth.get_width() == colorizer_processed_frame.get_width());

    rs2::video_frame to_disp_processed_frame = org_depth.apply_filter(to_disp);
    REQUIRE_FALSE(to_disp_processed_frame.is<rs2::frameset>());
    REQUIRE(to_disp_processed_frame.get_profile().stream_type() == RS2_STREAM_DEPTH);
    bool is_disp = to_disp_processed_frame.get_profile().format() == RS2_FORMAT_DISPARITY16 ||
        to_disp_processed_frame.get_profile().format() == RS2_FORMAT_DISPARITY32;
    if (supports_disparity)
    {
        REQUIRE(is_disp);
        REQUIRE(org_depth.get_width() == to_disp_processed_frame.get_width());
    }

    rs2::video_frame from_disp_processed_frame = org_depth.apply_filter(from_disp);//should bypass
    REQUIRE_FALSE(from_disp_processed_frame.is<rs2::frameset>());
    REQUIRE(from_disp_processed_frame.get_profile().stream_type() == RS2_STREAM_DEPTH);
    REQUIRE(from_disp_processed_frame.get_profile().format() == RS2_FORMAT_Z16);
    REQUIRE(org_depth.get_width() == from_disp_processed_frame.get_width());

    pipe.stop();
}

TEST_CASE("Post-Processing processing pipe", "[post-processing-filters]")
{
    rs2::context ctx;

    if (!make_context(SECTION_FROM_TEST_NAME, &ctx))
        return;

    rs2::temporal_filter temporal;
    rs2::hole_filling_filter hole_filling;
    rs2::spatial_filter spatial;
    rs2::decimation_filter decimation(4);
    rs2::align aligner(RS2_STREAM_COLOR);
    rs2::colorizer depth_colorizer;
    rs2::disparity_transform to_disp;
    rs2::disparity_transform from_disp(false);
    rs2::pointcloud pc(RS2_STREAM_DEPTH);

    rs2::config cfg;
    cfg.enable_all_streams();

    rs2::pipeline pipe(ctx);
    auto profile = pipe.start(cfg);

    bool supports_disparity = false;
    for (auto s : profile.get_device().query_sensors())
    {
        if (s.supports(RS2_OPTION_STEREO_BASELINE))
        {
            supports_disparity = true;
            break;
        }
    }

    rs2::frameset original = pipe.wait_for_frames();

    rs2::frameset full_pipe;
    int run_for = 10;
    std::set<int> uids;
    int uid_count = 0;
    while (run_for--)
    {
        full_pipe = pipe.wait_for_frames();
        full_pipe = full_pipe.apply_filter(decimation);
        full_pipe = full_pipe.apply_filter(to_disp);
        full_pipe = full_pipe.apply_filter(spatial);
        full_pipe = full_pipe.apply_filter(temporal);
        full_pipe = full_pipe.apply_filter(from_disp);
        full_pipe = full_pipe.apply_filter(aligner);
        full_pipe = full_pipe.apply_filter(hole_filling);
        full_pipe = full_pipe.apply_filter(depth_colorizer);
        full_pipe = full_pipe.apply_filter(pc);

        //printf("test frame:\n");
        full_pipe.foreach([&](const rs2::frame& f) {
            uids.insert(f.get_profile().unique_id());
            //printf("stream: %s, format: %d, uid: %d\n", f.get_profile().stream_name().c_str(), f.get_profile().format(), f.get_profile().unique_id());
        });
        if (uid_count == 0)
            uid_count = uids.size();
        REQUIRE(uid_count == uids.size());
    }

    REQUIRE(is_subset(original, full_pipe));
    REQUIRE_THROWS(is_subset(full_pipe, original));
    pipe.stop();
}

TEST_CASE("Align from recording with illigal input", "[software-device][align]") {
    rs2::context ctx;
    if (!make_context(SECTION_FROM_TEST_NAME, &ctx))
        return;

    std::string folder_name = get_folder_path(special_folder::temp_folder);
    const std::string filename = folder_name + "single_depth_color_640x480.bag";
    REQUIRE(file_exists(filename));
    auto dev = ctx.load_device(filename);

    rs2::syncer sync;
    std::vector<rs2::sensor> sensors = dev.query_sensors();
    REQUIRE(sensors.size() == 2);
    for (auto s : sensors)
    {
        if (s.get_stream_profiles().front().stream_type() == rs2_stream::RS2_STREAM_DEPTH) continue;
        REQUIRE_NOTHROW(s.open(s.get_stream_profiles().front()));
        REQUIRE_NOTHROW(s.start(sync));   
    }

    //try to process align with no depth frame, expect for a gracefull error
    rs2::frameset frameset = sync.wait_for_frames(200);
    rs2::align align(rs2_stream::RS2_STREAM_ANY);
    auto processed = align.process(frameset);

}

TEST_CASE("Align from recording with different resolutions", "[software-device][align]") {
    rs2::context ctx;
    if (!make_context(SECTION_FROM_TEST_NAME, &ctx))
        return;
    rs2::align align(rs2_stream::RS2_STREAM_DEPTH);
    std::vector<rs2::frame> frames;
    //try to process align with no depth frame, expect for a gracefull error
    rs2::processing_block frame_processor(
        [&](rs2::frameset data, // Input frameset (from the pipeline)
            rs2::frame_source& source) // Frame pool that can allocate new frames
    {
        frames.push_back(data);
        if (frames.size() == 2)
        {
            source.frame_ready(source.allocate_composite_frame(frames));
            frames.clear();
        }
    });
    rs2::frame_queue postprocessed_frames;
    frame_processor >> postprocessed_frames;

    bool processing = true;
    std::thread video_processing_thread([&]() {
        while (processing)
        {
            rs2::frameset composite_fs;
            if (postprocessed_frames.try_wait_for_frame(&composite_fs))
            {
                const int EXPECTED_WIDTH[] = { 1280, 848, 640 }, EXPECTED_HIGHT[] = { 720, 480, 480 };
                static int frame_number = 0;
                auto fs = align.process(composite_fs);
                auto cf = fs.get_color_frame();
                CAPTURE(cf.get_width());
                REQUIRE((cf.get_width() == EXPECTED_WIDTH[frame_number]));
                CAPTURE(cf.get_height());
                REQUIRE((cf.get_height() == EXPECTED_HIGHT[frame_number]));
                frame_number++;
            }
        }
    });

    std::vector<std::string> file_names = { "depth_1280x720_color_1920x1080.bag", "depth_848x480_color_960x540.bag", "depth_640x480_color_424x240.bag" };
    for (auto f : file_names)
    {
        std::cout << "Running file : " << f << std::endl;
        //std::string folder_name = get_folder_path(special_folder::temp_folder);
        std::string folder_name("C:\\Users\\ibelkin\\Documents\\recordings\\");
        const std::string filepath = folder_name + f;
        REQUIRE(file_exists(filepath));
        auto dev = ctx.load_device(filepath);

        rs2::syncer sync;
        std::vector<rs2::sensor> sensors = dev.query_sensors();
        REQUIRE(sensors.size() == 2);
        for (auto s : sensors)
        {
            REQUIRE_NOTHROW(s.open(s.get_stream_profiles().front()));
            REQUIRE_NOTHROW(s.start(sync));
        }

        rs2::frameset fs;
        while (sync.try_wait_for_frames(&fs))
        {
            std::cout << "Recieved frame with size : " << fs.size() << std::endl;
            frame_processor.invoke(fs);
        }
        frames.clear();
    }

    processing = false;
    if (video_processing_thread.joinable())
        video_processing_thread.join();
}

TEST_CASE("Align from recording with different resolutions2", "[software-device][align]") {
    rs2::context ctx;
    if (!make_context(SECTION_FROM_TEST_NAME, &ctx))
        return;
    rs2::align align(rs2_stream::RS2_STREAM_DEPTH);
    std::vector<rs2::frame> frames;
    //try to process align with no depth frame, expect for a gracefull error
    rs2::processing_block frame_processor(
        [&](rs2::frameset data, // Input frameset (from the pipeline)
            rs2::frame_source& source) // Frame pool that can allocate new frames
    {
        frames.push_back(data);
        if (frames.size() == 2)
        {
            source.frame_ready(source.allocate_composite_frame(frames));
            frames.clear();
        }
    });
    rs2::frame_queue postprocessed_frames;
    frame_processor >> postprocessed_frames;

    bool processing = true;
    std::thread video_processing_thread([&]() {
        while (processing)
        {
            rs2::frameset composite_fs;
            if (postprocessed_frames.try_wait_for_frame(&composite_fs))
            {
                const int EXPECTED_WIDTH[] = { 1280, 848, 640 }, EXPECTED_HIGHT[] = { 720, 480, 480 };
                static int frame_number = 0;
                auto fs = align.process(composite_fs);
                auto cf = fs.get_color_frame();
                CAPTURE(cf.get_width());
                REQUIRE((cf.get_width() == EXPECTED_WIDTH[frame_number]));
                CAPTURE(cf.get_height());
                REQUIRE((cf.get_height() == EXPECTED_HIGHT[frame_number]));
                frame_number++;
            }
        }
    });

    std::vector<std::string> file_names = { "depth_1280x720_color_1920x1080.bag", "depth_848x480_color_960x540.bag", "depth_640x480_color_424x240.bag" };
    for (auto f : file_names)
    {
        std::cout << "Running file : " << f << std::endl;
        //std::string folder_name = get_folder_path(special_folder::temp_folder);
        std::string folder_name("C:\\Users\\ibelkin\\Documents\\recordings\\");
        const std::string filepath = folder_name + f;
        REQUIRE(file_exists(filepath));
        auto dev = ctx.load_device(filepath);

        rs2::syncer sync;
        std::vector<rs2::sensor> sensors = dev.query_sensors();
        REQUIRE(sensors.size() == 2);
        for (auto s : sensors)
        {
            REQUIRE_NOTHROW(s.open(s.get_stream_profiles().front()));
            REQUIRE_NOTHROW(s.start(sync));
        }

        rs2::frameset fs;
        while (sync.try_wait_for_frames(&fs))
        {
            std::cout << "Recieved frame with size : " << fs.size() << std::endl;
            frame_processor.invoke(fs);
        }
        frames.clear();
    }

    processing = false;
    if (video_processing_thread.joinable())
        video_processing_thread.join();
}

std::vector<rs2::stream_profile> get_stream_profiles(rs2::software_sensor ss, rs2::video_stream_profile c, rs2::video_stream_profile d, rs2::video_stream_profile ir)
{
    std::vector<rs2::stream_profile> stream_profiles;
    rs2_video_stream depth_stream = { d.stream_type(), d.stream_index(), d.unique_id(), d.width(), d.height(), d.fps(), d.height(), d.format(), d.get_intrinsics() };
    stream_profiles.push_back(ss.add_video_stream(depth_stream));

    rs2_video_stream ir_stream = { ir.stream_type(), ir.stream_index(), ir.unique_id(), ir.width(), ir.height(), ir.fps(), d.height(), ir.format(), ir.get_intrinsics() };
    stream_profiles.push_back(ss.add_video_stream(ir_stream));

    rs2_video_stream color_stream = { c.stream_type(), c.stream_index(), c.unique_id(), c.width(), c.height(), c.fps(), d.height(), c.format(), c.get_intrinsics() };
    stream_profiles.push_back(ss.add_video_stream(color_stream));

    return stream_profiles;
}

std::string get_sensor_name(rs2::video_stream_profile c, rs2::video_stream_profile d)
{
    std::string dres = std::to_string(d.width()) + "x" + std::to_string(d.height());
    std::string cres = std::to_string(c.width()) + "x" + std::to_string(c.height());
    std::string name = "depth_ir_" + dres + "_color_" + std::to_string(c.format()) + "_" + cres;
    return name;
}

typedef struct _sw_context
{
    rs2::software_device sdev;
    std::map<std::string, rs2::software_sensor> sw_sensors;
    std::map<std::string, rs2::syncer> sw_syncers;
    std::map<std::string, std::vector<rs2::stream_profile>> sw_stream_profiles;
} sw_context;

sw_context init_sw_device(std::vector<std::vector<rs2::stream_profile>> depth_ir_profiles,
    std::vector<rs2::video_stream_profile> color_profiles)
{
    sw_context sctx;
    for (auto p1 : depth_ir_profiles)
    {
        for (auto p2 : color_profiles)
        {
            auto d = p1[0].as<rs2::video_stream_profile>();
            auto ir = p1[1].as<rs2::video_stream_profile>();
            auto c = p2.as<rs2::video_stream_profile>();

            std::string name = get_sensor_name(c, d);
            sctx.sw_sensors[name] = sctx.sdev.add_sensor(name);

            sctx.sw_stream_profiles[name] = get_stream_profiles(sctx.sw_sensors[name], c, d, ir);

            rs2::syncer ssync;
            sctx.sw_sensors[name].open(sctx.sw_stream_profiles[name]);
            sctx.sw_sensors[name].start(ssync);
            sctx.sw_syncers[name] = ssync;
        }
    }
    return sctx;
}

void record_sw_frame(rs2::frameset fs, sw_context sctx)
{
    auto fd = fs.get_depth_frame();
    auto fir = fs.get_infrared_frame();
    auto fc = fs.get_color_frame();

    auto d = fd.get_profile().as<rs2::video_stream_profile>();
    auto ir = fir.get_profile().as<rs2::video_stream_profile>();
    auto c = fc.get_profile().as<rs2::video_stream_profile>();

    auto ss = sctx.sw_sensors[get_sensor_name(c, d)];
    auto stream_profiles = sctx.sw_stream_profiles[get_sensor_name(c, d)];

    ss.on_video_frame({ (void*)fd.get_data(), [](void*) {}, d.width(), fd.get_bits_per_pixel(), fd.get_timestamp(), fd.get_frame_timestamp_domain(), static_cast<int>(fd.get_frame_number()), stream_profiles[0] });
    ss.on_video_frame({ (void*)fir.get_data(), [](void*) {}, ir.width(), fir.get_bits_per_pixel(), fir.get_timestamp(), fir.get_frame_timestamp_domain(), static_cast<int>(fir.get_frame_number()), stream_profiles[1] });
    ss.on_video_frame({ (void*)fc.get_data(), [](void*) {}, c.width(), fc.get_bits_per_pixel(), fc.get_timestamp(), fc.get_frame_timestamp_domain(), static_cast<int>(fc.get_frame_number()), stream_profiles[2] });

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
    auto profiles1 = sensors[0].get_stream_profiles();

    std::vector<rs2::video_stream_profile> depth_profiles;
    for (auto p : profiles1)
    {
        if (p.stream_type() == rs2_stream::RS2_STREAM_DEPTH)
        {
            auto pv = p.as<rs2::video_stream_profile>();
            if (!std::any_of(depth_profiles.begin(), depth_profiles.end(), [&](rs2::video_stream_profile i)
            {
                return i.height() == pv.height() &&
                    i.width() == pv.width() &&
                    i.format() == pv.format();
            }))
            {
                depth_profiles.push_back(pv);
            }
        }
    }

    std::vector<std::vector<rs2::stream_profile>> depth_ir_profiles;
    for (auto d : depth_profiles)
    {
        for (auto p : profiles1)
        {
            if (p.stream_type() == rs2_stream::RS2_STREAM_INFRARED && p.stream_index() == 1)
            {
                auto pv = p.as<rs2::video_stream_profile>();
                if (d.height() == pv.height() && d.width() == pv.width())
                {
                    std::vector<rs2::stream_profile> tmp;
                    tmp.push_back(d);
                    tmp.push_back(pv);
                    depth_ir_profiles.push_back(tmp);
                    break;
                }
            }
        }
    }

    auto profiles2 = sensors[1].get_stream_profiles();
    std::vector<rs2::video_stream_profile> color_profiles;
    for (auto p : profiles2)
    {
        if (p.stream_type() == rs2_stream::RS2_STREAM_COLOR)
        {
            auto pv = p.as<rs2::video_stream_profile>();
            if (!std::any_of(color_profiles.begin(), color_profiles.end(), [&](rs2::video_stream_profile i)
            {
                return i.height() == pv.height() &&
                    i.width() == pv.width();
            }))
            {
                color_profiles.push_back(pv);
            }
        }
    }

    auto sctx = init_sw_device(depth_ir_profiles, color_profiles);

    rs2::recorder recorder("recording.bag", sctx.sdev);

    for (auto p1 : depth_ir_profiles)
    {
        for (auto p2 : color_profiles)
        {
            rs2::syncer sync;
            sensors[0].open(p1);
            sensors[1].open(p2);

            sensors[0].start(sync);
            sensors[1].start(sync);

            while (true)
            {
                auto fs = sync.wait_for_frames(200);
                if (fs.size() == 3)
                {
                    static int i = 1;
                    std::cout << "Yeah" << i << "\n";
                    i++;

                    record_sw_frame(fs, sctx);
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

rs2::video_stream_profile get_stream_profile(std::vector<rs2::stream_profile> profiles, rs2_stream type)
{
    for (auto p : profiles)
    {
        if (p.stream_type() == type)
            return p.as<rs2::video_stream_profile>();
    }
    throw std::runtime_error("No such profile");
}

sw_context init_sw_device(std::vector<rs2::sensor> sensors)
{
    sw_context sctx;
    for (auto s : sensors)
    {
        auto profiles = s.get_stream_profiles();
        auto d = get_stream_profile(profiles, rs2_stream::RS2_STREAM_DEPTH);
        auto ir = get_stream_profile(profiles, rs2_stream::RS2_STREAM_INFRARED);
        auto c = get_stream_profile(profiles, rs2_stream::RS2_STREAM_COLOR);

        std::string name = get_sensor_name(c, d);
        sctx.sw_sensors[name] = sctx.sdev.add_sensor(name);

        sctx.sw_stream_profiles[name] = get_stream_profiles(sctx.sw_sensors[name], c, d, ir);;

        rs2::syncer ssync;
        sctx.sw_sensors[name].open(s.get_stream_profiles());
        sctx.sw_sensors[name].start(ssync);
        sctx.sw_syncers[name] = ssync;
    }
    return sctx;
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
        std::cout << "new frame " << frames.size() << std::endl;
        if (frames.size() == 3)
        {
            std::cout << "new frame in" << std::endl;
            source.frame_ready(source.allocate_composite_frame(frames));
            frames.clear();
        }
    });

    rs2::frame_queue postprocessed_frames;
    frame_processor >> postprocessed_frames;

    bool processing = true;
    int recorded_frame_size = 1;
    std::thread video_processing_thread([&]() {
        while (processing)
        {
            rs2::frameset composite_fs;
            if (postprocessed_frames.try_wait_for_frame(&composite_fs))
            {
                composite_fs.keep();
                composite_frames.push_back(composite_fs);
                std::cout << "recorded frame : " << recorded_frame_size << std::endl;
                recorded_frame_size++;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });

    std::mutex sensor_lock;
    int frames_size = 1;
    for (auto s : sensors)
    {
        s.open(s.get_stream_profiles());
        s.start([&](rs2::frame f)
        {
            std::lock_guard<std::mutex> lock(sensor_lock);
            std::cout << "sensor :" << frames_size << " frame: " << f.get_profile().stream_name() << ", number: " << f.get_frame_number() << std::endl;
            f.keep();
            frame_processor.invoke(f);
            frames_size++;
        });
    }

    while (recorded_frame_size <= sensors.size())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    for (auto s : sensors)
    {
        s.stop();
        s.close();
    }

    processing = false;
    if (video_processing_thread.joinable())
        video_processing_thread.join();

    return composite_frames;
}

TEST_CASE("Play software-device all resolutions", "[software-device][record]")
{
    rs2::context ctx;
    if (!make_context(SECTION_FROM_TEST_NAME, &ctx))
        return;
    auto dev = ctx.load_device("recording.bag");
    dev.set_real_time(false);

    std::vector<rs2::sensor> sensors = dev.query_sensors();
    auto sctx = init_sw_device(sensors);
    rs2::recorder recorder("recording_res.bag", sctx.sdev);

    rs2::align align(rs2_stream::RS2_STREAM_DEPTH);
    for (auto f : get_composite_frames(sensors))
    {
        record_sw_frame(align.process(f), sctx);
    }
}

bool validate_ppf_results(const rs2::frame& result_frame, const rs2::frame& reference_frame, size_t frame_idx, std::string output_file)
{
    std::vector<uint16_t> diff2orig;
    std::vector<uint16_t> diff2ref;

    // Basic sanity scenario with no filters applied.
    // validating domain transform in/out conversion. Requiring input=output
    //bool domain_transform_only = (reference_data.downsample_scale == 1) &&
    //    (!reference_data.spatial_filter) && (!reference_data.temporal_filter);

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

    auto pixels = result_profile.width()*result_profile.height();
    diff2ref.resize(pixels);

    // Pixel-by-pixel comparison of the resulted filtered depth vs data ercorded with external tool
    auto v1 = reinterpret_cast<const uint16_t*>(result_frame.get_data());
    auto v2 = reinterpret_cast<const uint16_t*>(reference_frame.get_data());

    for (auto i = 0; i < pixels; i++)
    {
        if (i == 620192 || i == 531184)
        {
            auto v11 = *v1;
            auto v22 = *v2;
        }
        uint16_t diff = std::abs(*v1++ - *v2++);
        diff2ref[i] = diff;
    }

    // Validate the filters
    // The differences between the reference code and librealsense implementation are byte-compared below
    return profile_diffs(output_file/*"./Filterstransform.txt"*/, diff2ref, 0.f, 0, frame_idx);
}

void saveImage(rs2::video_frame& frame, std::string header)
{
    std::ofstream save_file("Image_" + header +
        ".pgm", std::ofstream::binary);

    std::string pgm_header = "P5 " + header + " 255\n";
    save_file.write(pgm_header.c_str(), pgm_header.size());
    save_file.write((char*)frame.get_data(), frame.get_height() * frame.get_width() * frame.get_bits_per_pixel());
    save_file.close();
}

TEST_CASE("Compare software-device all resolutions", "[software-device][record]")
{
    rs2::context ctx;
    if (!make_context(SECTION_FROM_TEST_NAME, &ctx))
        return;
    auto dev = ctx.load_device("recording.bag");
    dev.set_real_time(false);

    std::vector<rs2::sensor> sensors = dev.query_sensors();

    auto frames = get_composite_frames(sensors);

    auto res_dev = ctx.load_device("recording_res.bag");
    dev.set_real_time(false);

    std::vector<rs2::sensor> res_sensors = res_dev.query_sensors();

    auto res_frames = get_composite_frames(res_sensors);

    rs2::align align(rs2_stream::RS2_STREAM_DEPTH);
    for (int i = 0; i < frames.size(); i++)
    {
        //compare
        auto fs_res = align.process(frames[i]);
        saveImage(frames[i].get_depth_frame(), "orig");
        saveImage(fs_res.get_depth_frame(), "res");
        saveImage(res_frames[i].get_depth_frame(), "ref");
        validate_ppf_results(fs_res.get_depth_frame(), res_frames[i].get_depth_frame(), i, "./align.txt");
        validate_ppf_results(fs_res.get_color_frame(), res_frames[i].get_color_frame(), i, "./align.txt");
        validate_ppf_results(fs_res.get_infrared_frame(), res_frames[i].get_infrared_frame(), i, "./align.txt");

    }
}
