//! This module mimics some of the functions find in pixi that works with the data types
//! there but work with the project model types instead.
use itertools::Either;
use pixi_build_types as pbt;
use rattler_conda_types::Platform;

pub trait TargetSelectorExt {
    /// Does the target selector match the platform?
    fn matches(&self, platform: Platform) -> bool;
}

impl TargetSelectorExt for pbt::TargetSelectorV1 {
    fn matches(&self, platform: Platform) -> bool {
        match self {
            pbt::TargetSelectorV1::Platform(p) => p == &platform.to_string(),
            pbt::TargetSelectorV1::Linux => platform.is_linux(),
            pbt::TargetSelectorV1::Unix => platform.is_unix(),
            pbt::TargetSelectorV1::Win => platform.is_windows(),
            pbt::TargetSelectorV1::MacOs => platform.is_osx(),
        }
    }
}

/// Extends the  type with additional functionality.
pub trait TargetExt<'a, Selector: TargetSelectorExt, Target>
where
    Selector: 'a,
    Target: 'a,
{
    /// Returns the default target.
    fn default_target(&self) -> &Target;

    /// Returns all targets
    fn targets(&'a self) -> impl Iterator<Item = (&'a Selector, &'a Target)>;

    /// Resolve the target for the given platform.
    fn resolve(&'a self, platform: Option<Platform>) -> impl Iterator<Item = &'a Target> {
        if let Some(platform) = platform {
            let iter = std::iter::once(self.default_target()).chain(self.targets().filter_map(
                move |(selector, target)| {
                    if selector.matches(platform) {
                        Some(target)
                    } else {
                        None
                    }
                },
            ));
            Either::Right(iter)
        } else {
            Either::Left(std::iter::once(self.default_target()))
        }
    }
}

impl<'a> TargetExt<'a, pbt::TargetSelectorV1, pbt::TargetV1> for pbt::TargetsV1 {
    fn default_target(&self) -> &pbt::TargetV1 {
        &self.default_target
    }

    fn targets(&'a self) -> impl Iterator<Item = (&'a pbt::TargetSelectorV1, &'a pbt::TargetV1)> {
        self.targets.iter()
    }
}
