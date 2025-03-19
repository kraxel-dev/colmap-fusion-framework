// Copyright (c) 2023, ETH Zurich and UNC Chapel Hill.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//
//     * Neither the name of ETH Zurich and UNC Chapel Hill nor the names of
//       its contributors may be used to endorse or promote products derived
//       from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#pragma once

#include <QtCore>
#include <QtGui>
#include <QtWidgets>
#include <memory>

#include "tightly_coupled_fusion/controllers/incremental_fusion_pipeline.h"
#include <colmap/scene/reconstruction.h>
#include <colmap/sensor/bitmap.h>
#include <colmap/ui/automatic_reconstruction_widget.h>
#include <colmap/ui/bundle_adjustment_widget.h>
#include <colmap/ui/database_management_widget.h>
#include <colmap/ui/dense_reconstruction_widget.h>
#include <colmap/ui/feature_extraction_widget.h>
#include <colmap/ui/feature_matching_widget.h>
#include <colmap/ui/license_widget.h>
#include <colmap/ui/log_widget.h>
#include <colmap/ui/match_matrix_widget.h>
#include <colmap/ui/model_viewer_widget.h>
#include <colmap/ui/project_widget.h>
#include <colmap/ui/reconstruction_manager_widget.h>
#include <colmap/ui/reconstruction_options_widget.h>
#include <colmap/ui/reconstruction_stats_widget.h>
#include <colmap/ui/render_options_widget.h>
#include <colmap/ui/undistortion_widget.h>
#include <colmap/util/controller_thread.h>

namespace tcf {

class MainWindow : public QMainWindow {
 public:
  explicit MainWindow(const colmap::OptionManager& options);

  void ImportReconstruction(const std::string& path);

 protected:
  void closeEvent(QCloseEvent* event);

 private:
  friend class AutomaticReconstructionWidget;
  friend class BundleAdjustmentWidget;
  friend class DenseReconstructionWidget;

  void CreateWidgets();
  void CreateActions();
  void CreateMenus();
  void CreateToolbar();
  void CreateStatusbar();
  void CreateControllers();

  void ProjectNew();
  bool ProjectOpen();
  void ProjectEdit();
  void ProjectSave();
  void ProjectSaveAs();
  void Import();
  void ImportFrom();
  void Export();
  void ExportAll();
  void ExportAs();
  void ExportAsText();

  void FeatureExtraction();
  void FeatureMatching();
  void DatabaseManagement();

  void AutomaticReconstruction();

  void ReconstructionStart();
  void ReconstructionStep();
  void ReconstructionPause();
  void ReconstructionReset();
  void ReconstructionOptions();
  void ReconstructionFinish();
  void ReconstructionNormalize();
  bool ReconstructionOverwrite();

  void BundleAdjustment();
  void DenseReconstruction();

  void Render();
  void RenderNow();
  void RenderToggle();
  void RenderOptions();
  void RenderSelectedReconstruction();
  void RenderClear();

  void SelectReconstructionIdx(size_t);
  size_t SelectedReconstructionIdx();
  bool HasSelectedReconstruction();
  bool IsSelectedReconstructionValid();

  void GrabImage();
  void UndistortImages();

  void ReconstructionStats();
  void MatchMatrix();
  void ShowLog();
  void ExtractColors();

  void SetOptions();
  void ResetOptions();

  void About();
  void Documentation();
  void Support();

  void ShowInvalidProjectError();
  void UpdateTimer();

  void EnableBlockingActions();
  void DisableBlockingActions();

  void UpdateWindowTitle();

  colmap::OptionManager options_;

  std::shared_ptr<colmap::ReconstructionManager> reconstruction_manager_;
  // -------------------- Custom incremental fusion pipeline
  std::unique_ptr<colmap::ControllerThread<tcf::IncrementalFusionPipeline>> mapper_controller_;

  colmap::Timer timer_;

  colmap::ModelViewerWidget* model_viewer_widget_;
  colmap::ProjectWidget* project_widget_;
  colmap::FeatureExtractionWidget* feature_extraction_widget_;
  colmap::FeatureMatchingWidget* feature_matching_widget_;
  colmap::DatabaseManagementWidget* database_management_widget_;
  colmap::AutomaticReconstructionWidget* automatic_reconstruction_widget_;
  colmap::ReconstructionOptionsWidget* reconstruction_options_widget_;
  colmap::BundleAdjustmentWidget* bundle_adjustment_widget_;
  colmap::DenseReconstructionWidget* dense_reconstruction_widget_;
  colmap::RenderOptionsWidget* render_options_widget_;
  colmap::LogWidget* log_widget_;
  colmap::UndistortionWidget* undistortion_widget_;
  colmap::ReconstructionManagerWidget* reconstruction_manager_widget_;
  colmap::ReconstructionStatsWidget* reconstruction_stats_widget_;
  colmap::MatchMatrixWidget* match_matrix_widget_;
  colmap::LicenseWidget* license_widget_;
  colmap::ThreadControlWidget* thread_control_widget_;

  QToolBar* file_toolbar_;
  QToolBar* preprocessing_toolbar_;
  QToolBar* reconstruction_toolbar_;
  QToolBar* render_toolbar_;
  QToolBar* extras_toolbar_;

  QDockWidget* dock_log_widget_;

  QTimer* statusbar_timer_;
  QLabel* statusbar_timer_label_;

  QAction* action_project_new_;
  QAction* action_project_open_;
  QAction* action_project_edit_;
  QAction* action_project_save_;
  QAction* action_project_save_as_;
  QAction* action_import_;
  QAction* action_import_from_;
  QAction* action_export_;
  QAction* action_export_all_;
  QAction* action_export_as_;
  QAction* action_export_as_text_;
  QAction* action_quit_;

  QAction* action_feature_extraction_;
  QAction* action_feature_matching_;
  QAction* action_database_management_;

  QAction* action_automatic_reconstruction_;

  QAction* action_reconstruction_start_;
  QAction* action_reconstruction_step_;
  QAction* action_reconstruction_pause_;
  QAction* action_reconstruction_reset_;
  QAction* action_reconstruction_finish_;
  QAction* action_reconstruction_normalize_;
  QAction* action_reconstruction_options_;

  QAction* action_bundle_adjustment_;
  QAction* action_dense_reconstruction_;

  QAction* action_render_;
  QAction* action_render_now_;
  QAction* action_render_toggle_;
  QAction* action_render_reset_view_;
  QAction* action_render_options_;

  QAction* action_reconstruction_stats_;
  QAction* action_match_matrix_;
  QAction* action_log_show_;
  QAction* action_grab_image_;
  QAction* action_grab_movie_;
  QAction* action_undistort_;
  QAction* action_extract_colors_;
  QAction* action_set_options_;
  QAction* action_reset_options_;

  QAction* action_about_;
  QAction* action_documentation_;
  QAction* action_support_;
  QAction* action_license_;

  std::vector<QAction*> blocking_actions_;

  // Necessary for OS X to avoid duplicate closeEvents.
  bool window_closed_;
};

}  // namespace tcf
