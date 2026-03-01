# More resources on COLMAP and Fuma cli options

Unfortunately, many of COLMAP's parametrizable options for all steps across the mapping pipeline leave a lot of room for interpretation. The sheer amount of options doesn't necessary help.

At least, for the options introduced by this repo to perform fusion mapping, explanations can be found in the source code:

- Check `FusionGraphBundleAdjustmentOptions` cpp struct in file: [include/tightly_coupled_fusion/estimators/bundle_adjustment.h](../include/tightly_coupled_fusion/estimators/bundle_adjustment.h)
- Check `FusionMapperOptions` cpp struct in [include/tightly_coupled_fusion/sfm/incremental_mapper.h](../include/tightly_coupled_fusion/sfm/incremental_mapper.h)
- Check [include/fusion_helper/rr_sfm_logger.h](../include/fusion_helper/rr_sfm_logger.h) for rerun visualization args
- Check [include/fusion_helper/frame_align_utils.h](../include/fusion_helper/frame_align_utils.h) to check args that perform COLMAP model transformations during active reconstruction as convenience for later evaluation
- [include/fusion_helper/cov_utils.h](../include/fusion_helper/cov_utils.h)
- [io.h](../include/fusion_helper/io.h) in helpers to change the hardcoded digit precision after which the timestamps are cut to make matching (in seconds) easier

TODO: manual description of some of the most useful COLMAP params.