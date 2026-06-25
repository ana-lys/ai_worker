#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to ffw_collision_checker__msg__CollisionCheck
/// CollisionCheck.msg
/// Message for publishing collision check results

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CollisionCheck {
    /// Whether any collision is detected (distance < 0)
    pub in_collision: bool,

    /// Array of distances for all contact pairs
    /// Negative values indicate penetration depth
    pub distances: Vec<f64>,

    /// Names of first geometry in each contact pair
    pub geom1_names: Vec<std::string::String>,

    /// Names of second geometry in each contact pair
    pub geom2_names: Vec<std::string::String>,

}



impl Default for CollisionCheck {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::CollisionCheck::default())
  }
}

impl rosidl_runtime_rs::Message for CollisionCheck {
  type RmwMsg = super::msg::rmw::CollisionCheck;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        in_collision: msg.in_collision,
        distances: msg.distances.into(),
        geom1_names: msg.geom1_names
          .into_iter()
          .map(|elem| elem.as_str().into())
          .collect(),
        geom2_names: msg.geom2_names
          .into_iter()
          .map(|elem| elem.as_str().into())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      in_collision: msg.in_collision,
        distances: msg.distances.as_slice().into(),
        geom1_names: msg.geom1_names
          .iter()
          .map(|elem| elem.as_str().into())
          .collect(),
        geom2_names: msg.geom2_names
          .iter()
          .map(|elem| elem.as_str().into())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      in_collision: msg.in_collision,
      distances: msg.distances
          .into_iter()
          .collect(),
      geom1_names: msg.geom1_names
          .into_iter()
          .map(|elem| elem.to_string())
          .collect(),
      geom2_names: msg.geom2_names
          .into_iter()
          .map(|elem| elem.to_string())
          .collect(),
    }
  }
}


