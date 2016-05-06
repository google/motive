// Copyright 2016 Google Inc. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cassert>
#include <cstdlib>
#include "anim_generated.h"
#include "fplbase/asset_manager.h"
#include "fplbase/input.h"
#include "fplbase/renderer.h"
#include "fplbase/utilities.h"
#include "fplutil/file_utils.h"
#include "mathfu/matrix.h"
#include "mathfu/matrix_4x4.h"
#include "motive/anim.h"
#include "motive/engine.h"
#include "motive/io/flatbuffers.h"
#include "motive/math/angle.h"
#include "motive/motivator.h"

using mathfu::mat3;
using mathfu::mat4;
using mathfu::vec2;
using mathfu::vec3;
using mathfu::kZeros3f;
using motive::Angle;
using motive::AngleToVectorSystem;
using motive::MotiveTime;

enum AnimationState {
  kAnimationStateNoAnimation,
  kAnimationStatePaused,
  kAnimationStateStepOneFrame,
  kAnimationStateAnimating,
};

// Command line arguments.
struct ViewerArgs {
  ViewerArgs()
      : playback_rate(1.0f),
        bone_idx(-1),
        coordinate_system(motive::kAngleToVectorXY) {}

  std::string anim_file;
  std::string mesh_file;
  std::string out_file;
  float playback_rate;
  int bone_idx;
  AngleToVectorSystem coordinate_system;
};

// Camera is always pointed at the origin, where the model is.
struct Camera {
  Camera(AngleToVectorSystem coordinate_system, float aspect_ratio,
         const vec3& model_min_position, const vec3& model_max_position);

  AngleToVectorSystem coordinate_system;
  float distance;
  Angle latitude;
  Angle longitude;
  float aspect_ratio;
  float z_near;
  float z_far;
  vec3 target;
};

// Time used by anim_pipeline is 1000 ticks per second.
static const double kMotiveAnimTicksPerSecond = 1000.0;
static const float kMaxLatitude = motive::kDegreesToRadians * 85.0f;
static const float kLatitudeSensitivity = motive::kDegreesToRadians * 0.15f;
static const float kLongitudeSensitivity = motive::kDegreesToRadians * 0.3f;
static const float kDistanceSensitivity = 0.1f;
static const float kTargetSensitivity = 0.001f;
static const float kPlaybackRateSensitivity = 0.01f;
static const float kMinPlaybackRate = 0.01f;
static const float kMaxPlaybackRate = 10.0f;
static const float kPersepectiveFieldOfViewY =
    45.0f * motive::kDegreesToRadians;

// No skinning, so just draw on screen.
static const char kVertexShaderMonolythic[] =
    "attribute vec4 aPosition;\n"
    "attribute vec2 aTexCoord;\n"
    "attribute vec3 aNormal;\n"
    "\n"
    "varying vec2 vTexCoord;\n"
    "varying vec3 vNormal;\n"
    "\n"
    "uniform mat4 model_view_projection;\n"
    "\n"
    "void main()\n"
    "{\n"
    "  gl_Position = model_view_projection * aPosition;\n"
    "  vTexCoord = aTexCoord;\n"
    "  vNormal = aNormal;\n"
    "}\n";

// See fplbase's skinning.glslv_h for a description of OneBoneSkinnedPosition().
static const char kVertexShaderSkinned[] =
    "const int kMaxNumShaderBones = 256;\n"
    "const int kNumVec4sInAffineTransform = 3;\n"
    "\n"
    "attribute vec4 aBoneIndices;\n"
    "attribute vec4 aPosition;\n"
    "attribute vec2 aTexCoord;\n"
    "attribute vec3 aNormal;\n"
    "\n"
    "varying vec2 vTexCoord;\n"
    "varying vec3 vNormal;\n"
    "\n"
    "uniform mat4 model_view_projection;\n"
    "uniform vec4 bone_transforms[kMaxNumShaderBones *\n"
    "                             kNumVec4sInAffineTransform];\n"
    "\n"
    "mat4 BoneTransformMatrixTranspose(int i) {\n"
    "  return mat4(bone_transforms[i * 3 + 0],\n"
    "              bone_transforms[i * 3 + 1],\n"
    "              bone_transforms[i * 3 + 2],\n"
    "              vec4(0, 0, 0, 1));\n"
    "}\n"
    "\n"
    "vec4 OneBoneSkinnedPosition(vec4 position) {\n"
    "  return position * BoneTransformMatrixTranspose(int(aBoneIndices.x));\n"
    "}\n"
    "\n"
    "void main()\n"
    "{\n"
    "  vec4 model_position = OneBoneSkinnedPosition(aPosition);\n"
    "  gl_Position = model_view_projection * model_position;\n"
    "  vTexCoord = aTexCoord;\n"
    "  vNormal = (model_view_projection * vec4(aNormal, 0)).xyz;\n"
    "}\n";

// When no textures exist, draw using two Phong light sources.
static const char kFragmentShaderPhong[] =
    "const vec4 kAmbientColor = vec4(0.3, 0.3, 0.3, 0);\n"
    "const vec4 kDiffuseColor1 = vec4(0.4, 0.0, 0, 0);\n"
    "const vec4 kDiffuseColor2 = vec4(0, 0, 0.6, 0);\n"
    "const vec3 kLightDirection1 = vec3(0.577, 0.577, 0.577);\n"
    "const vec3 kLightDirection2 = vec3(0.267, 0.535, 0.802);\n"
    "varying vec3 vNormal;\n"
    "\n"
    "void main()\n"
    "{\n"
    "  float diffuse1 = abs(dot(vNormal, kLightDirection1));\n"
    "  float diffuse2 = abs(dot(vNormal, kLightDirection2));\n"
    "  gl_FragColor = kAmbientColor + diffuse1 * kDiffuseColor1 +\n"
    "                 diffuse2 * kDiffuseColor2;\n"
    "}\n";

// Read texture and use no light sources.
static const char kFragmentShaderFlatTexture[] =
    "varying mediump vec2 vTexCoord;\n"
    "uniform sampler2D texture_unit_0;\n"
    "void main()\n"
    "{\n"
    "  lowp vec4 texture_color = texture2D(texture_unit_0, vTexCoord);\n"
    "  gl_FragColor = texture_color;\n"
    "}\n";

static const char kCommandLineUsage[] =
    "Usage: fplviewer [-x|-y|-z] [-f FRAMES_PER_SECOND] [-b BONE_IDX]\n"
    "                 [-o ANIM_DEBUG_OUTPUT.csv]\n"
    "                 [ANIMATION_FILE.motiveanim] [MESH_FILE.fplmesh]\n"
    "\n"
    "Tool to preview and debug FPL meshes and Motive animations.\n"
    "\n"
    "Options:\n"
    "  -x, --x-up            Use the x-axis as up.\n"
    "  -y, --y-up            Use the y-axis as up.\n"
    "  -z, --z-up            Use the z-axis as up. This is the default value.\n"
    "  -r, --playbackrate    Playback rate of the animation.\n"
    "                        1 is the authored speed, and the default.\n"
    "                        0.5 is half speed. 2.0 is double speed.\n"
    "  -b, --bone            Bone index to debug. Transform data is output\n"
    "                        every frame for this bone. Bone indices can be\n"
    "                        found with the -i switch on mesh_pipeline.\n"
    "  -o, --out             Comma-separated output file to which we write\n"
    "                        all the animation channels. Open in any\n"
    "                        spreadsheet to examine and graph.\n"
    "                        If unspecified, we write to ANIMATION_FILE.csv\n"
    "  MESH_FILE             Mesh to display. If unspecified,\n"
    "                        tries to load ANIMATION_FILE.fplmesh.\n"
    "  ANIMATION_FILE        The name of the motiveanim file to preview.\n"
    "\n";

static const char kControls[] =
    "\n"
    "fplviewer controls\n"
    "==================\n"
    "   left-mouse button drag -- rotate camera about model\n"
    "   right-mouse button drag -- zoom camera in and out\n"
    "   middle-mouse button drag -- translate camera\n"
    "   left and right cursor -- speed-up and slow-down anim playback rate\n"
    "   space bar -- pause animation\n"
    "   right cursor while paused -- single step animation\n"
    "\n";

static bool ParseCommandLine(int argc, char* argv[], ViewerArgs* args) {
  // Parse options. The last argument can't be an option.
  int i = 1;  // Skip the first argument, which is the executible name.
  for (; i < argc - 1; ++i) {
    // Exit once we run out of options.
    const std::string option = argv[i];
    if (option[0] != '-') break;

    // Parse options that have no value.
    if (option == "-x" || option == "--x-up") {
      args->coordinate_system = motive::kAngleToVectorYZ;
      continue;
    }

    if (option == "-y" || option == "--y-up") {
      args->coordinate_system = motive::kAngleToVectorXZ;
      continue;
    }

    if (option == "-z" || option == "--z-up") {
      args->coordinate_system = motive::kAngleToVectorXY;
      continue;
    }

    // Parse options have have a value in the next argument.
    const bool valid_value = i + 1 < argc - 1 && argv[i + 1][0] != '-';
    if (!valid_value) {
      printf("ERROR: Must specify a value for option %s\n\n", option.c_str());
      return false;
    }
    const std::string option_value = argv[i + 1];
    i++;

    if (option == "-r" || option == "--playbackrate") {
      args->playback_rate = atof(option_value.c_str());
      if (args->playback_rate <= 0.0f || args->playback_rate > 1000.0f) {
        printf("ERROR: Invalid playback rate %f.\n\n", args->playback_rate);
        return false;
      }
      continue;
    }

    if (option == "-b" || option == "--bone") {
      args->bone_idx = atoi(option_value.c_str());
      if (args->bone_idx < 0 || args->bone_idx > 10000) {
        printf("ERROR: Invalid bone index %d.\n\n", args->bone_idx);
        return false;
      }
      continue;
    }

    if (option == "-o" || option == "--out") {
      args->out_file = option_value;
      continue;
    }
  }

  // Parse mesh and animation file names.
  for (; i < argc; ++i) {
    const std::string file_name = argv[i];
    const std::string file_extension = fplutil::FileExtension(file_name);

    if (file_extension == "motiveanim") {
      args->anim_file = file_name;

      // Initialize other file names based on anim file name.
      const std::string anim_base = fplutil::RemoveExtensionFromName(file_name);
      if (args->mesh_file.length() == 0) {
        args->mesh_file = anim_base + ".fplmesh";
      }
      if (args->out_file.length() == 0) {
        args->out_file = anim_base + ".csv";
      }

    } else if (file_extension == "fplmesh") {
      args->mesh_file = file_name;

    } else {
      printf(
          "ERROR: Unknown file type %s."
          " Must be a `motiveanim` or `fplmesh` file.\n\n",
          file_name.c_str());
      return false;
    }
  }

  // We can't do anything without a mesh file.
  if (args->mesh_file.length() == 0) {
    printf("ERROR: Must specify an fplmesh or motiveanim file.\n\n");
    return false;
  }

  return true;
}

Camera::Camera(AngleToVectorSystem coordinate_system, float aspect_ratio,
               const vec3& model_min_position, const vec3& model_max_position)
    : coordinate_system(coordinate_system),
      distance(0.0f),
      latitude(0.0f),
      longitude(0.0f),
      aspect_ratio(aspect_ratio),
      z_near(1.0f),
      z_far(100.0f),
      target(kZeros3f) {
  // Set the initial target to point up from the middle a bit, so that the
  // model is vertically centered. We don't make it horizontally centered,
  // since it could move left or right a lot while animating.
  const vec3 avg_position = 0.5f * (model_min_position + model_max_position);
  target = avg_position * VectorSystemUp(coordinate_system);

  //         distance
  // -----------------------
  //  \__kOnScreenAngle    .
  //     \__               .
  //        \__            .
  //           \__         . model_width
  //              \__      .
  //                 \__   .
  //                    \__.
  // tan(kOnScreenAngle) = model_width / distance
  //            distance = model_width / tan(kOnScreenAngle)
  const float kWidthToCameraDistance =
      1.0f / tan(motive::kDegreesToRadians * 30.0f);
  const vec3 model_width3 =
      vec3::Max(vec3::Max(vec3::Max(model_min_position, model_max_position),
                          -model_min_position),
                -model_max_position);
  const float model_width =
      std::max(std::max(model_width3.x(), model_width3.y()), model_width3.z());
  distance = model_width * kWidthToCameraDistance;

  // Set near and far planes based on the distance to the model.
  z_near = distance / 32.0f;
  z_far = distance * 256.0f;
}

static bool LoadAnim(const char* anim_name, motive::RigAnim* anim) {
  std::string anim_buf;
  const bool load_ok = fplbase::LoadFile(anim_name, &anim_buf);
  if (!load_ok) {
    // printf("Failed to load animation file\n");
    return false;
  }
  const motive::RigAnimFb* anim_fb = motive::GetRigAnimFb(anim_buf.c_str());
  motive::RigAnimFromFlatBuffers(*anim_fb, anim_name, anim);
  return true;
}

static vec3 CameraPosition(const Camera& camera) {
  const vec3 camera_unit_sphere = motive::LatitudeAndLongitudeToUnitSphere(
      camera.latitude, camera.longitude, camera.coordinate_system);
  const vec3 camera_position =
      camera.distance * camera_unit_sphere + camera.target;
  return camera_position;
}

static vec3 CameraRight(const Camera& camera) {
  const vec3 camera_position = CameraPosition(camera);
  const vec3 left = vec3::CrossProduct(
      camera_position, VectorSystemUp(camera.coordinate_system));
  return -left;
}

static mat4 CalculateMvp(const Camera& camera) {
  const vec3 camera_position = CameraPosition(camera);
  const mat4 persp =
      mat4::Perspective(kPersepectiveFieldOfViewY, camera.aspect_ratio,
                        camera.z_near, camera.z_far, -1.0f);
  const mat4 lookat =
      mat4::LookAt(camera.target, camera_position,
                   VectorSystemUp(camera.coordinate_system), -1.0f);
  return persp * lookat;
}

static void UpdateCamera(fplbase::InputSystem* input, Camera* camera) {
  const fplbase::InputPointer& pointer = input->get_pointers()[0];
  if (!pointer.used) return;

  // Update latitude and longitude: left mouse drag.
  if (input->GetButton(fplbase::K_POINTER1).is_down() &&
      pointer.mousedelta != mathfu::kZeros2i) {
    const float lat = camera->latitude.ToRadians() +
                      pointer.mousedelta.y() * kLatitudeSensitivity;
    camera->latitude = Angle(mathfu::Clamp(lat, -kMaxLatitude, kMaxLatitude));
    camera->longitude += Angle(pointer.mousedelta.x() * kLongitudeSensitivity);
  }

  // Update distance: right mouse drag.
  // (Unfortunately, InputSystem doesn't support scroll wheels yet)
  if (input->GetButton(fplbase::K_POINTER3).is_down() &&
      pointer.mousedelta != mathfu::kZeros2i) {
    const float dist = camera->distance +
                       (pointer.mousedelta.x() + pointer.mousedelta.y()) *
                           kDistanceSensitivity;
    camera->distance = std::max(dist, camera->z_near);
  }

  // Update target: middle mouse drag.
  if (input->GetButton(fplbase::K_POINTER2).is_down() &&
      pointer.mousedelta != mathfu::kZeros2i) {
    const vec3 right = CameraRight(*camera);
    const vec3 up = VectorSystemUp(camera->coordinate_system);
    const vec2 dist = kTargetSensitivity * vec2(pointer.mousedelta);
    camera->target += dist[0] * right + dist[1] * up;
  }
}

static AnimationState UpdateAnimationState(AnimationState animation_state,
                                           fplbase::InputSystem* input) {
  // Do nothing if we have no animations.
  if (animation_state == kAnimationStateNoAnimation)
    return kAnimationStateNoAnimation;

  // Return to paused state after single-step.
  if (animation_state == kAnimationStateStepOneFrame) {
    animation_state = kAnimationStatePaused;
  }

  // When paused, allow single step with right cursor key.
  if (animation_state == kAnimationStatePaused &&
      input->GetButton(fplbase::FPLK_RIGHT).went_down())
    return kAnimationStateStepOneFrame;

  // Toggle pause on space-bar.
  if (input->GetButton(fplbase::FPLK_SPACE).went_down())
    return animation_state == kAnimationStatePaused ? kAnimationStateAnimating
                                                    : kAnimationStatePaused;

  // No change.
  return animation_state;
}

static bool UpdatePlaybackRate(AnimationState animation_state,
                               fplbase::InputSystem* input,
                               float* playback_rate) {
  if (animation_state != kAnimationStateAnimating) return false;

  const bool left = input->GetButton(fplbase::FPLK_LEFT).is_down();
  if (!left && !input->GetButton(fplbase::FPLK_RIGHT).is_down()) return false;

  const float sign = left ? -1.0f : 1.0f;
  const float orig_rate = *playback_rate;
  *playback_rate = mathfu::Clamp(orig_rate + sign * kPlaybackRateSensitivity,
                                 kMinPlaybackRate, kMaxPlaybackRate);
  return orig_rate != *playback_rate;
}

static float AspectRatio(const fplbase::Renderer& renderer) {
  return static_cast<float>(renderer.window_size().x()) /
         renderer.window_size().y();
}

extern "C" int FPL_main(int argc, char* argv[]) {
  // Output command-line usage if arguments invalid.
  ViewerArgs args;
  const bool valid_args = ParseCommandLine(argc, argv, &args);
  if (!valid_args) {
    printf(kCommandLineUsage);
    return 1;
  }

  // Initialize subsystems.
  motive::SplineInit::Register();
  motive::MatrixInit::Register();
  motive::RigInit::Register();

  fplbase::Renderer renderer;
  renderer.Initialize(mathfu::vec2i(800, 600), "Motive viewer");

  fplbase::InputSystem input;
  input.Initialize();

  // Load animation file.
  fplbase::AssetManager asset_manager(renderer);
  motive::RigAnim anim;
  if (args.anim_file.length() > 0) {
    const bool anim_result = LoadAnim(args.anim_file.c_str(), &anim);
    if (!anim_result) {
      printf("ERROR: Could not load animation file `%s`\n\n",
             args.anim_file.c_str());
      return 1;
    }
  }

  // Load mesh file.
  fplbase::Mesh* mesh = asset_manager.LoadMesh(args.mesh_file.c_str());
  if (!mesh) {
    printf("ERROR: Could not load mesh file `%s`\n\n", args.mesh_file.c_str());
    return 1;
  }
  asset_manager.StartLoadingTextures();

  // Wait for assets to load.
  while (!asset_manager.TryFinalize()) {
  }

  // Compile shaders.
  const bool mesh_has_textures = mesh->GetMaterial(0) != nullptr &&
                                 mesh->GetMaterial(0)->textures().size() > 0;
  const bool mesh_is_skinned = mesh->num_shader_bones() > 0;
  const char* fragment_shader =
      mesh_has_textures ? kFragmentShaderFlatTexture : kFragmentShaderPhong;
  const char* vertex_shader =
      mesh_is_skinned ? kVertexShaderSkinned : kVertexShaderMonolythic;
  fplbase::Shader* shader =
      renderer.CompileAndLinkShader(vertex_shader, fragment_shader);
  if (!shader) {
    printf("ERROR: Could not compile viewer shaders.\n\n%s\n\n",
           renderer.last_error().c_str());
    return 1;
  }

  // UX values.
  Camera camera(args.coordinate_system, AspectRatio(renderer),
                mesh->min_position(), mesh->max_position());
  float playback_rate = args.playback_rate;
  AnimationState animation_state = anim.NumBones() > 0
                                       ? kAnimationStateAnimating
                                       : kAnimationStateNoAnimation;

  // Ensure the mesh and the animation are compatible.
  const bool compatible = animation_state == kAnimationStateNoAnimation ||
                          anim.NumBones() == 1 ||
                          motive::RigInit::MatchesHierarchy(
                              anim, mesh->bone_parents(), mesh->num_bones());
  if (!compatible) {
    printf(
        "ERROR: Animation %s and mesh %s do not have a compatible hierarchy.\n"
        "       Did you run mesh_pipeline with -h?\n\n",
        args.anim_file.c_str(), args.mesh_file.c_str());
    return 1;
  }

  // Initialize the RigMotivator to animate the `mesh` according to `anim`.
  motive::MotiveEngine engine;
  motive::RigMotivator motivator;
  if (animation_state != kAnimationStateNoAnimation) {
    const motive::RigInit init(
        anim, mesh->bone_transforms(), mesh->bone_parents(),
        static_cast<motive::BoneIndex>(mesh->num_bones()));
    motivator.Initialize(init, &engine);
    motivator.BlendToAnim(
        anim, motive::SplinePlayback(0.0f, true, args.playback_rate));
  }

  // Create an array to hold the bone matrices that are pushed to the shader.
  std::vector<mathfu::AffineTransform> shader_transforms(
      mesh->num_shader_bones());

  // Output how-to-use message, with commands.
  printf(kControls);

  // Open file that recieves debug info.
  FILE* out_file = nullptr;
  if (animation_state != kAnimationStateNoAnimation) {
    out_file = fopen(args.out_file.c_str(), "w");
    const char* out_file_msg = out_file ? "Open `%s` in a spreadsheet for "
                                          "values on every animation channel.\n"
                                        : "WARNING: Could not open output file "
                                          "for animation debugging `%s`\n";
    printf(out_file_msg, args.out_file.c_str());
  }

  // Output header if we're debugging all the bones, not just one specific one.
  if (out_file) {
    fprintf(out_file, "%s", motivator.CsvHeaderForDebugging().c_str());
  }

  while (!(input.exit_requested() ||
           input.GetButton(fplbase::FPLK_AC_BACK).went_down())) {
    // Process input.
    UpdateCamera(&input, &camera);
    animation_state = UpdateAnimationState(animation_state, &input);
    if (UpdatePlaybackRate(animation_state, &input, &playback_rate)) {
      motivator.SetPlaybackRate(playback_rate);
    }

    // Update sub-systems.
    renderer.AdvanceFrame(input.minimized(), input.Time());
    input.AdvanceFrame(&renderer.window_size());
    if (animation_state == kAnimationStateAnimating ||
        animation_state == kAnimationStateStepOneFrame) {
      engine.AdvanceFrame(static_cast<MotiveTime>(kMotiveAnimTicksPerSecond *
                                                  input.DeltaTime()));
    }

    // Output debug information for this frame.
    if (out_file) {
      fprintf(out_file, "%s\n", motivator.CsvValuesForDebugging().c_str());
    }
    if (args.bone_idx >= 0) {
      printf("%s",
             motivator.LocalTransformsForDebugging(args.bone_idx).c_str());
    }

    // Render the mesh.
    renderer.ClearFrameBuffer(mathfu::vec4(0.0, 0.0f, 0.0, 1.0f));
    mat4 mvp = CalculateMvp(camera);

    if (mesh_is_skinned) {
      // Set the bone trasform uniforms.
      // If animated, grab from animation. Otherwise, use default values.
      const mathfu::AffineTransform* bone_transforms =
          animation_state == kAnimationStateNoAnimation
              ? mesh->bone_global_transforms()
              : motivator.GlobalTransforms();
      mesh->GatherShaderTransforms(bone_transforms, shader_transforms.data());
      renderer.SetBoneTransforms(shader_transforms.data(),
                                 mesh->num_shader_bones());
    } else if (animation_state != kAnimationStateNoAnimation) {
      // We're animating but there are no bones uniforms. Just move the
      // root bone.
      mvp *= mat4::FromAffineTransform(motivator.GlobalTransforms()[0]);
    }

    renderer.set_model_view_projection(mvp);
    shader->Set(renderer);
    mesh->Render(renderer);
  }

  asset_manager.ClearAllAssets();
  renderer.ShutDown();

  return 0;
}
