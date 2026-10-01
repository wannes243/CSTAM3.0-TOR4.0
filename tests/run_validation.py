"""Run isolated package suites and the portable realtime pipeline.

Each Python suite runs in a fresh process. Live ROS/Webots verification uses
tests/live_pipeline.py and never substitutes ROS messages or robot devices.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

ROOT=Path(__file__).parents[1]
OPS='tests/packages/test_operations.py::'
SUITES={
    'fabtino_core':['tests/test_ekf.py','tests/test_motion_model.py','tests/test_navigation.py',
                    'tests/test_static_points.py','tests/test_denied_zones.py','tests/test_delivery.py',
                    'tests/test_scan_matcher.py','tests/test_map_localizer.py','tests/test_transforms.py',
                    OPS+'test_core_rejects_malformed_import_without_mutating_any_map',
                    OPS+'test_default_known_map_matching_recovers_asymmetric_room'],
    'fabtino_webots_bridge':['tests/test_lidar_mask.py',
                    OPS+'test_bridge_keeps_scan_reset_and_newest_motion_when_motor_updates_flood',
                    OPS+'test_bridge_reset_service_resets_estimator_and_map'],
    'fabtino_localization':['tests/test_rotation_mapping.py','tests/test_wall_contact.py',
                    OPS+'test_localization_reset_consumes_new_absolute_encoders_and_resets_yaw',
                    OPS+'test_import_worker_starts_only_for_fresh_stationary_scan'],
    'fabtino_mapping':['tests/test_mapping_runtime.py',
                    'tests/packages/test_mapping_features.py',
                    OPS+'test_mapping_clear_has_ack_and_does_not_reintegrate_queued_old_scans',
                    OPS+'test_configure_map_changes_ros_grid_and_completes',
                    OPS+'test_successful_import_commits_after_localization_and_preserves_imu_heading'],
    'fabtino_navigation':['tests/test_goal_control.py','tests/test_navigation_flow.py','tests/test_arrival_chain.py',
                    'tests/packages/test_navigation_features.py',
                    OPS+'test_navigation_and_safety_commands_reach_bridge_after_full_topic_fanout'],
    'fabtino_safety':['tests/test_safety_arbitration.py','tests/packages/test_navigation_features.py::test_safety_stops_swept_navigation_entry_but_allows_manual_recovery'],
    'fabtino_viewer':['tests/test_viewer_pointcloud.py','tests/test_drive_chain.py',
                    'tests/packages/test_viewer_features.py',
                    OPS+'test_viewer_drive_reenables_lidar_and_preserves_both_turn_signs',
                    OPS+'test_viewer_reports_status_even_before_first_odometry',
                    OPS+'test_viewer_does_not_replay_stale_pose_or_map_after_reset',
                    OPS+'test_import_error_releases_viewer_and_preserves_previous_map',
                    OPS+'test_operation_timeout_cancels_workers_and_allows_new_drive',
                    OPS+'test_new_clear_cancels_pending_import_in_both_nodes'],
    'fabtino_bringup':['tests/test_world_integration.py','tests/packages/test_bringup.py'],
    'controller':['tests/packages/test_controller.py'],
    'pipeline':['tests/pipeline'],
}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--package',choices=['all',*SUITES],default='all')
    parser.add_argument('--browser',action='store_true',help='Also run Chrome interaction and full network tests')
    args=parser.parse_args()
    selected=list(SUITES) if args.package=='all' else [args.package]
    results=[];folder=ROOT/'tests/results';folder.mkdir(exist_ok=True)
    for name in selected:
        junit=folder/f'{name}-junit.xml'
        start=time.monotonic()
        completed=subprocess.run([sys.executable,'-m','pytest',*SUITES[name],'-q',f'--junitxml={junit}'],cwd=ROOT)
        result=dict(suite=name,result='passed' if completed.returncode==0 else 'failed',
                    elapsed_s=round(time.monotonic()-start,3))
        if junit.is_file():
            xml=ET.parse(junit).getroot()
            result.update({key:sum(int(suite.get(key,0)) for suite in xml.findall('testsuite'))
                           for key in ('tests','failures','errors','skipped')})
        results.append(result)
    js=['tests/test_navigation_ui.cjs','tests/test_map_orientation.cjs','tests/test_map_bundle.cjs']
    commands=[['node','--test',*js]] if args.package in ('all','fabtino_viewer') else []
    if args.browser:commands+=[['node',file] for file in ('tests/controls_browser.cjs','tests/navigation_browser.cjs','tests/pipeline_browser.cjs','tests/features_browser.cjs')]
    for command in commands:
        start=time.monotonic();completed=subprocess.run(command,cwd=ROOT)
        results.append(dict(suite=' '.join(command),result='passed' if completed.returncode==0 else 'failed',elapsed_s=round(time.monotonic()-start,3)))
    report=dict(scope='Portable: ROS topic/message doubles and simulated Webots devices; real TCP/WS and Chrome when requested',
                result='passed' if all(item['result']=='passed' for item in results) else 'failed',suites=results)
    destination=folder/('package_validation.json' if args.package=='all' else f'{args.package}-validation.json')
    destination.write_text(json.dumps(report,indent=2)+'\n')
    print(f"Validation {report['result']}: {destination}")
    return 0 if report['result']=='passed' else 1


if __name__=='__main__':raise SystemExit(main())
