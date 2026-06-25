// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from ffw_collision_checker:msg/CollisionCheck.idl
// generated code does not contain a copyright notice

#include "ffw_collision_checker/msg/detail/collision_check__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_ffw_collision_checker
const rosidl_type_hash_t *
ffw_collision_checker__msg__CollisionCheck__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x9f, 0x50, 0x95, 0xca, 0xc0, 0x1b, 0x99, 0x84,
      0xf5, 0xf4, 0x7f, 0x39, 0x37, 0x5f, 0xe3, 0xb1,
      0xf8, 0x41, 0xf3, 0x10, 0x43, 0x34, 0x3b, 0xb7,
      0xc5, 0x5e, 0x09, 0x39, 0x5d, 0x26, 0x71, 0xaf,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char ffw_collision_checker__msg__CollisionCheck__TYPE_NAME[] = "ffw_collision_checker/msg/CollisionCheck";

// Define type names, field names, and default values
static char ffw_collision_checker__msg__CollisionCheck__FIELD_NAME__in_collision[] = "in_collision";
static char ffw_collision_checker__msg__CollisionCheck__FIELD_NAME__distances[] = "distances";
static char ffw_collision_checker__msg__CollisionCheck__FIELD_NAME__geom1_names[] = "geom1_names";
static char ffw_collision_checker__msg__CollisionCheck__FIELD_NAME__geom2_names[] = "geom2_names";

static rosidl_runtime_c__type_description__Field ffw_collision_checker__msg__CollisionCheck__FIELDS[] = {
  {
    {ffw_collision_checker__msg__CollisionCheck__FIELD_NAME__in_collision, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ffw_collision_checker__msg__CollisionCheck__FIELD_NAME__distances, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ffw_collision_checker__msg__CollisionCheck__FIELD_NAME__geom1_names, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ffw_collision_checker__msg__CollisionCheck__FIELD_NAME__geom2_names, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
ffw_collision_checker__msg__CollisionCheck__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {ffw_collision_checker__msg__CollisionCheck__TYPE_NAME, 40, 40},
      {ffw_collision_checker__msg__CollisionCheck__FIELDS, 4, 4},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# CollisionCheck.msg\n"
  "# Message for publishing collision check results\n"
  "\n"
  "# Whether any collision is detected (distance < 0)\n"
  "bool in_collision\n"
  "\n"
  "# Array of distances for all contact pairs\n"
  "# Negative values indicate penetration depth\n"
  "float64[] distances\n"
  "\n"
  "# Names of first geometry in each contact pair\n"
  "string[] geom1_names\n"
  "\n"
  "# Names of second geometry in each contact pair\n"
  "string[] geom2_names";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
ffw_collision_checker__msg__CollisionCheck__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {ffw_collision_checker__msg__CollisionCheck__TYPE_NAME, 40, 40},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 387, 387},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
ffw_collision_checker__msg__CollisionCheck__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *ffw_collision_checker__msg__CollisionCheck__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
