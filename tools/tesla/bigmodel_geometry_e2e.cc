// Native camera metadata -> calibrated warp -> actual NV12 LUT -> transmitted matrix proof.
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include "frame_meta.h"
#include "warp_pack.h"

int main(int argc, char** argv) {
  MetaProvider provider;
  chipmunk::WarpLut lut;
  const float rpy[3] = {0.01f, -0.02f, 0.005f};
  const float physical_height = 1.22f;
  const float offset = argc > 1 ? std::strtof(argv[1], nullptr) : 0.f;
  for (int i = 0; i < 12; ++i) provider.set_rpy(rpy, physical_height, offset);
  const double filtered_offset = offset * (1. - std::pow(0.9, 12));
  std::puts("{\"cases\":[");
  bool comma = false;
  for (CameraModel sensor : {CameraModel::AR_OX, CameraModel::OS04C10}) {
    for (bool resized : {false, true}) {
      if (sensor == CameraModel::OS04C10 && !resized) continue;
      const int width = resized ? 1344 : 1928, height = resized ? 760 : 1208, stride = width + 32;
      for (bool wide : {false, true}) {
        assert(provider.set_camera_geometry(wide, sensor, width, height));
        float actual[9], expected[9], intrinsics[9];
        provider.warp(wide, actual);
        const double focal = sensor == CameraModel::OS04C10 ? (wide ? 425.25 : 1141.5) : (wide ? 567. : 2648.);
        const double sx = sensor == CameraModel::AR_OX ? double(width) / 1928 : 1.;
        const double sy = sensor == CameraModel::AR_OX ? double(height) / 1208 : 1.;
        const float reference_k[9] = {float(focal*sx), 0, width*.5f, 0, float(focal*sy), height*.5f, 0, 0, 1};
        assert(camera_intrinsics(sensor, wide, width, height, intrinsics));
        assert(std::memcmp(reference_k, intrinsics, sizeof intrinsics) == 0);
        get_warp_matrix(rpy, reference_k, wide, expected);
        const float shear = filtered_offset / physical_height;
        const float translation = -filtered_offset / physical_height * (reference_k[5] - reference_k[4] * std::tan(rpy[1]));
        for (int i = 0; i < 3; ++i) expected[i] += shear * expected[i+3] + translation * expected[i+6];
        for (int i = 0; i < 9; ++i) {
          if (std::abs(expected[i] - actual[i]) >= 0.0002f) std::fprintf(stderr, "sensor=%d wide=%d offset=%g i=%d expected=%g actual=%g\n", int(sensor), wide, offset, i, expected[i], actual[i]);
          assert(std::abs(expected[i] - actual[i]) < 0.0002f);
        }
        if (sensor == CameraModel::OS04C10 && offset == 0.f) {
          float legacy[9];
          get_warp_matrix(rpy, wide ? kWideRoadIntrinsics : kNarrowRoadIntrinsics, wide, legacy);
          assert(std::memcmp(legacy, actual, sizeof actual) == 0);
        }
        std::vector<uint8_t> input(stride * height * 3 / 2);
        for (size_t i = 0; i < input.size(); ++i) input[i] = uint8_t((i * 37 + i / stride * 11) % 251);
        chipmunk::Nv12View src{input.data(), input.data() + stride*height, width, height, stride, stride};
        constexpr int out_stride = 544, out_height = chipmunk::kModelH;
        std::vector<uint8_t> got(out_stride*out_height*3/2, 0xee), ref(got.size(), 0xee);
        chipmunk::Nv12Out dst{got.data(), got.data()+out_stride*out_height, out_stride, out_stride};
        chipmunk::Nv12Out oracle{ref.data(), ref.data()+out_stride*out_height, out_stride, out_stride};
        assert(lut.warp(src, actual, dst));
        assert(chipmunk::warpNv12(src, actual, oracle));
        assert(got == ref);
        bgm1::FrameHeader header, decoded;
        provider.fill(123, &header);
        assert(std::memcmp(wide ? header.warp_wide : header.warp_road, actual, sizeof actual) == 0);
        uint8_t wire[bgm1::kFrameHdrSize];
        bgm1::pack_frame_header(header, wire);
        assert(bgm1::parse_frame_header(wire, &decoded) == bgm1::Err::kOk);
        assert(std::memcmp(wide ? decoded.warp_wide : decoded.warp_road, actual, sizeof actual) == 0);
        if (comma) std::puts(",");
        comma = true;
        std::printf("{\"sensor\":\"%s\",\"width\":%d,\"height\":%d,\"offset\":%.9g,\"wide\":%s,\"passed\":true,\"warp\":[",
                    sensor == CameraModel::AR_OX ? "ar_ox" : "os04c10", width, height, offset, wide ? "true" : "false");
        for (int i = 0; i < 9; ++i) std::printf("%s%.9g", i ? "," : "", actual[i]);
        std::printf("]}");
      }
    }
  }
  for (auto sensor : {CameraModel::UNKNOWN, CameraModel::AR_OX, CameraModel::OS04C10}) {
    const int width = sensor == CameraModel::UNKNOWN ? 1344 : 1280;
    assert(!provider.set_camera_geometry(false, sensor, width, 760));
    float matrix[9]; provider.warp(false, matrix);
    for (float v : matrix) assert(v == 0.f);
  }
  assert(!provider.set_camera_geometry(false, CameraModel::OS04C10, 1928, 1208));
  assert(provider.set_camera_geometry(false, CameraModel::AR_OX, 1928, 1208));
  std::puts("],\"unknown_rejected\":true,\"recovery\":true,\"passed\":true}");
}
