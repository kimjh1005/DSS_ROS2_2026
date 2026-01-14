// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from livox_lio_sam:srv/Reset.idl
// generated code does not contain a copyright notice

#ifndef LIVOX_LIO_SAM__SRV__DETAIL__RESET__STRUCT_H_
#define LIVOX_LIO_SAM__SRV__DETAIL__RESET__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/Reset in the package livox_lio_sam.
typedef struct livox_lio_sam__srv__Reset_Request
{
  uint8_t structure_needs_at_least_one_member;
} livox_lio_sam__srv__Reset_Request;

// Struct for a sequence of livox_lio_sam__srv__Reset_Request.
typedef struct livox_lio_sam__srv__Reset_Request__Sequence
{
  livox_lio_sam__srv__Reset_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} livox_lio_sam__srv__Reset_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/Reset in the package livox_lio_sam.
typedef struct livox_lio_sam__srv__Reset_Response
{
  bool success;
} livox_lio_sam__srv__Reset_Response;

// Struct for a sequence of livox_lio_sam__srv__Reset_Response.
typedef struct livox_lio_sam__srv__Reset_Response__Sequence
{
  livox_lio_sam__srv__Reset_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} livox_lio_sam__srv__Reset_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // LIVOX_LIO_SAM__SRV__DETAIL__RESET__STRUCT_H_
