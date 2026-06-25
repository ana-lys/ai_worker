#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "ffw_collision_checker__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ffw_collision_checker__msg__CollisionCheck() -> *const std::ffi::c_void;
}

#[link(name = "ffw_collision_checker__rosidl_generator_c")]
extern "C" {
    fn ffw_collision_checker__msg__CollisionCheck__init(msg: *mut CollisionCheck) -> bool;
    fn ffw_collision_checker__msg__CollisionCheck__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<CollisionCheck>, size: usize) -> bool;
    fn ffw_collision_checker__msg__CollisionCheck__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<CollisionCheck>);
    fn ffw_collision_checker__msg__CollisionCheck__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<CollisionCheck>, out_seq: *mut rosidl_runtime_rs::Sequence<CollisionCheck>) -> bool;
}

// Corresponds to ffw_collision_checker__msg__CollisionCheck
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// CollisionCheck.msg
/// Message for publishing collision check results

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CollisionCheck {
    /// Whether any collision is detected (distance < 0)
    pub in_collision: bool,

    /// Array of distances for all contact pairs
    /// Negative values indicate penetration depth
    pub distances: rosidl_runtime_rs::Sequence<f64>,

    /// Names of first geometry in each contact pair
    pub geom1_names: rosidl_runtime_rs::Sequence<rosidl_runtime_rs::String>,

    /// Names of second geometry in each contact pair
    pub geom2_names: rosidl_runtime_rs::Sequence<rosidl_runtime_rs::String>,

}



impl Default for CollisionCheck {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ffw_collision_checker__msg__CollisionCheck__init(&mut msg as *mut _) {
        panic!("Call to ffw_collision_checker__msg__CollisionCheck__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for CollisionCheck {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ffw_collision_checker__msg__CollisionCheck__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ffw_collision_checker__msg__CollisionCheck__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ffw_collision_checker__msg__CollisionCheck__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for CollisionCheck {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for CollisionCheck where Self: Sized {
  const TYPE_NAME: &'static str = "ffw_collision_checker/msg/CollisionCheck";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ffw_collision_checker__msg__CollisionCheck() }
  }
}


