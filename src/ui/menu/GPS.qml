// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright © 2021-2022 Adrian <adrian.eddy at gmail>

import QtQuick
import QtQuick.Dialogs

import "../components/"

MenuItem {
    id: root;
    text: qsTr("GPS");
    iconName: "sync";
    objectName: "gps";

    function showExportDialog(extension, exportDataType) {
        const folder = filesystem.get_folder(window.videoArea.loadedFileUrl);
        const base = filesystem.get_filename(controller.input_file_url);
        const filename = filesystem.filename_with_extension(base, extension);
        exportFileDialog.selectedFile = filesystem.get_file_url(folder, filename, false);
        exportFileDialog.nameFilters = [extension.toUpperCase() + " (*." + extension + ")"];
        exportFileDialog.exportData = exportDataType;
        exportFileDialog.open2();
    }

    Button {
        text: qsTr("Load GPX");
        iconName: "file-empty"
        anchors.horizontalCenter: parent.horizontalCenter;
        onClicked: gpxDialog.open2();
    }
    FileDialog {
        id: gpxDialog;
        title: qsTr("Choose a GPX file");
        nameFilters: [qsTr("GPS Exchange Format") + " (*.gpx)"];
        type: "video";
        onAccepted: controller.load_gpx(selectedFile);
    }

    TableList {
        id: info;

        model: ({
            "GPX status": "---"
        })
    }
    Timer {
        id: gpxStatusTimer;
        interval: 500;
        running: true;
        repeat: true;
        onTriggered: {
            const s = controller.get_gpx_summary();
            if (s && s.loaded) {
                let msg = s.points + " pts";
                if (s.start_time !== undefined && s.epoch_end_s !== undefined) {
                    msg += ", " + new Date(s.start_time*1000).toISOString() + " → " + new Date(s.epoch_end_s*1000).toISOString();
                }
                if (s.overlap_ms !== undefined) {
                    const ms = Math.round(s.overlap_ms);
                    const mm = Math.floor(ms/60000);
                    const ss = Math.floor((ms%60000)/1000);
                    msg += ", overlap " + mm + ":" + ("0"+ss).slice(-2);
                }
                info.updateEntry("GPX status", msg);
            } else {
                info.updateEntry("GPX status", "---");
            }
        }
    }

    // GPS section (gated by has_gps())
    Column {
        id: gpsSection;
        width: parent.width;
        spacing: 6 * dpiScale;
        visible: controller.has_gps;
        property var gpsSettingsCache: null;
        function getGpsSettings() {
            try {
                gpsSettingsCache = JSON.parse(controller.get_gps_sync_settings());
            } catch (e) {
                gpsSettingsCache = {};
            }
            return gpsSettingsCache;
        }
        function withGpsSettings(mutator) {
            const settings = getGpsSettings();
            mutator(settings);
            controller.set_gps_sync_settings(JSON.stringify(settings));
        }
        function refreshGpsUI() {
            const settings = getGpsSettings();
            if (gpsSyncMethod) gpsSyncMethod.currentIndex = gpsSyncMethod.methodIndex(settings.method);
            if (gpsSpeedThreshold !== undefined) gpsSpeedThreshold.value = settings.speed_threshold;
            if (gpsMaxTimeOffset && settings.time_offset_range_s)
                gpsMaxTimeOffset.value = Math.max(settings.time_offset_range_s[1], -settings.time_offset_range_s[0]);
        }
        function getSafeNumberFieldValue(field, defaultValue) {
            return (field && typeof field.value === 'number') ? field.value : defaultValue;
        }
        Component.onCompleted: refreshGpsUI()

        Connections {
            target: controller;
            function onGps_changed(): void {
                gpsSyncMode.currentIndex = controller.gps_sync_mode;
                gpsSection.refreshGpsUI();
                gpsCoordsRow.refreshLatLon();
                if (gpsMapCheckbox.checked)
                    gpsMap.refreshMap();
            }
        }

        Label {
            position: Label.LeftPosition;
            text: qsTr("GPS sync mode")

            ComboBox {
                id: gpsSyncMode;
                model: [qsTr("Off"), qsTr("Auto"), qsTr("Manual")];
                font.pixelSize: 12 * dpiScale;
                width: parent.width;
                Component.onCompleted: currentIndex = controller.gps_sync_mode;
                onCurrentIndexChanged: controller.gps_sync_mode = currentIndex;
            }
        }
        CheckBox {
            id: gpsUseProcessedMotion;
            text: qsTr("Use processed motion for sync");
            visible: gpsSyncMode.currentIndex > 0; // Show for Auto and Manual modes
            checked: controller.gps_use_processed_motion;
            tooltip: qsTr("Use motion data stabilization and smoothing applied for GPS synchronization. Motion direction alignment can improve accuracy for head-mounted camera recordings.");
            onCheckedChanged: controller.gps_use_processed_motion = checked;
        }
        Label {
            position: Label.LeftPosition;
            text: qsTr("GPS sync method");
            visible: gpsSyncMode.currentIndex > 0; // Show for Auto and Manual modes

            ComboBox {
                id: gpsSyncMethod;
                visible: gpsSyncMode.currentIndex > 0; // Show for Auto and Manual modes
                // Centralized options to avoid hardcoded indices/strings in multiple places
                property var methodOptions: [
                    { label: qsTr("L1"),          value: "L1" },
                    { label: qsTr("Median L2"),   value: "MedL2" },
                    { label: qsTr("Trimmed L2"),  value: "TrimmedL2" }
                ]
                function methodIndex(value) {
                    for (var i = 0; i < methodOptions.length; i++)
                        if (methodOptions[i].value === value)
                            return i;
                    return 0;
                }
                function methodValue(index) {
                    return methodOptions[Math.max(0, Math.min(index, methodOptions.length - 1))].value;
                }
                model: methodOptions.map(function(opt) { return opt.label; });
                font.pixelSize: 12 * dpiScale;
                width: parent.width;
                tooltip: qsTr("Synchronization method: L1 uses the average absolute difference, Median L2 is more robust to outliers, Trimmed L2 uses the average of 90% of shortest squared distances.");
                onCurrentIndexChanged: gpsSection.withGpsSettings(s => s.method = methodValue(currentIndex))
            }
        }
        Label {
            position: Label.LeftPosition;
            text: qsTr("GPS speed threshold");
            visible: gpsSyncMode.currentIndex > 0; // Show for Auto and Manual modes

            NumberField {
                id: gpsSpeedThreshold;
                visible: gpsSyncMode.currentIndex > 0; // Show for Auto and Manual modes
                unit: "m/s";
                precision: 1;
                from: 0.1;
                to: 10.0;
                width: parent.width;
                tooltip: qsTr("Speed threshold for GPS masking. Regions with speed below this threshold will be shaded on the GPS chart, indicating unreliable GPS data.");
                onValueChanged: gpsSection.withGpsSettings(s => s.speed_threshold = value)
            }
        }
        Label {
            position: Label.LeftPosition;
            text: qsTr("Max. time offset");
            visible: gpsSyncMode.currentIndex === 1;

            NumberField {
                id: gpsMaxTimeOffset;
                visible: gpsSyncMode.currentIndex === 1;
                unit: qsTr("s");
                precision: 1;
                from: 0.0;
                to: 120.0;
                width: parent.width;
                tooltip: qsTr("Maximum time shift window for GPS/gyro synchronization search.");
                onValueChanged: gpsSection.withGpsSettings(s => s.time_offset_range_s = [-value, value])
            }
        }
        Label {
            visible: gpsSyncMode.currentIndex === 2;
            position: Label.LeftPosition;
            text: qsTr("GPS offset")

            NumberField {
                id: gpsOffset;
                width: Math.max(120 * dpiScale, root.width - 350 * dpiScale);
                height: 25 * dpiScale;
                precision: 2;
                unit: qsTr("s");
                value: controller.gps_offset_ms / 1000.0;
                enabled: gpsSyncMode.currentIndex === 2;
                onEditingFinished: if (gpsSyncMode.currentIndex === 2) controller.gps_offset_ms = value * 1000.0;

                Connections {
                    target: controller;
                    function onGps_changed() {
                        gpsOffset.value = controller.gps_offset_ms / 1000.0;
                    }
                }
            }
        }
        InfoMessageSmall {
            show: true;
            type: InfoMessage.Info;
            text: controller.gps_info_text
        }
        CheckBoxWithContent {
            id: gpsMapCheckbox;
            text: qsTr("GPS map");
            onCheckedChanged: {
                if (gpsMapCheckbox.checked) {
                    gpsMap.refreshMap();
                    // Initialize coordinates and dot immediately when map is shown
                    gpsCoordsRow.updatePositionAndCoords();
                }
                Qt.callLater(gpsMap.requestPaint);
            }
        }
        CheckBoxWithContent {
            id: gpsDistanceMarksCheckbox;
            visible: gpsMapCheckbox.checked;
            text: qsTr("Distance marks");
            Row {
                spacing: 6 * dpiScale;
                BasicText { text: qsTr("Step"); }
                NumberField {
                    id: gpsDistanceStep;
                    unit: "m";
                    precision: 1;
                    from: 1.0;
                    to: 1000.0;
                    width: Math.max(120 * dpiScale, root.width - 400 * dpiScale);
                    value: 10.0;
                    onValueChanged: if (gpsDistanceMarksCheckbox.checked) gpsMap.refreshDistanceMarks();
                }
            }
            onCheckedChanged: if (gpsMapCheckbox.checked) gpsMap.refreshDistanceMarks();
        }
        Row {
            id: gpsCoordsRow;
            visible: gpsMapCheckbox.checked;
            spacing: 8 * dpiScale;
            property var lastLatLon: [];
            function formatLatLon(lat, lon) {
                const latStr = (lat >= 0 ? "+" : "") + lat.toFixed(6);
                const lonStr = (lon >= 0 ? "+" : "") + lon.toFixed(6);
                return latStr + ", " + lonStr;
            }
			function currentTimestampUs() {
				if (window.videoArea && window.videoArea.timeline) {
					return Math.round(window.videoArea.timeline.position * window.videoArea.timeline.durationMs * 1000);
				}
				return 0;
			}
			function refreshLatLon(ts) {
				const t = (typeof ts === 'number') ? ts : currentTimestampUs();
				gpsCoordsRow.lastLatLon = controller.get_gps_current_latlon(t);
			}
			function updatePositionAndCoords(ts) {
				const timestamp = ts !== undefined ? ts : currentTimestampUs();
				refreshLatLon(timestamp);
				if (gpsMapCheckbox.checked) gpsMap.updatePosition(timestamp);
			}
            BasicText {
                id: gpsLatLonText;
                // Avoid binding width to parent which can cause polish loops
                wrapMode: Text.NoWrap;
                elide: Text.ElideRight;
                text: gpsCoordsRow.lastLatLon.length === 2 ? gpsCoordsRow.formatLatLon(gpsCoordsRow.lastLatLon[0], gpsCoordsRow.lastLatLon[1]) : qsTr("Lat, Lon: —");
                readonly property bool hasCoordinates: gpsCoordsRow.lastLatLon.length === 2;
                color: hasCoordinates ? styleAccentColor : styleTextColor;
                font.underline: hasCoordinates;

                MouseArea {
                    id: gpsCopyArea;
                    anchors.fill: parent;
                    enabled: parent.hasCoordinates;
                    cursorShape: enabled ? Qt.PointingHandCursor : Qt.ArrowCursor;
                    onClicked: if (parent.hasCoordinates) controller.copy_to_clipboard(parent.text);
                    hoverEnabled: true;
                }
                ToolTip {
                    visible: gpsLatLonText.hasCoordinates && gpsCopyArea.containsMouse;
                    text: qsTr("Click to copy");
                }
            }
			Connections {
				target: window.videoArea && window.videoArea.timeline ? window.videoArea.timeline : null;
				function onPositionChanged() {
					gpsCoordsRow.updatePositionAndCoords();
				}
			}
        }
        Canvas {
            id: gpsMap
            width: parent.width
            height: parent.width * 0.75
            visible: gpsMapCheckbox.checked
            property var poly: []
            property var currPos: []
            property var marks: []

            function refreshPolyline(): void {
                poly = controller.get_gps_trajectory_for_map(300);
                requestPaint();
            }
            function updatePosition(timestamp: real): void {
                currPos = controller.get_gps_current_pos_for_map(Math.round(timestamp));
                requestPaint();
            }
            function refreshDistanceMarks(): void {
                if (!gpsDistanceMarksCheckbox.checked) { marks = []; requestPaint(); return; }
                const step = gpsSection.getSafeNumberFieldValue(gpsDistanceStep, 10.0);
                const thr = gpsSection.getSafeNumberFieldValue(gpsSpeedThreshold, 0.0);
                marks = controller.get_gps_distance_marks_for_map(step, thr, 2000);
                requestPaint();
            }
            function refreshMap(): void {
                refreshPolyline();
                if (gpsDistanceMarksCheckbox.checked) refreshDistanceMarks();
            }

            onPaint: {
                if (!gpsMapCheckbox.checked) return;
                let ctx = getContext("2d");
                ctx.reset();

                // Uses as much canvas as possible while preserving uniform scaling
                let minx = 1e9, miny = 1e9, maxx = -1e9, maxy = -1e9;
                for (let i = 0; i < poly.length; i++) {
                    const px = poly[i][0]; const py = poly[i][1];
                    if (px < minx) minx = px; if (px > maxx) maxx = px;
                    if (py < miny) miny = py; if (py > maxy) maxy = py;
                }
                const pad = 6; // px margin
                const dx = Math.max(1e-6, maxx - minx);
                const dy = Math.max(1e-6, maxy - miny);
                const sx = (width  - 2 * pad) / dx;
                const sy = (height - 2 * pad) / dy;
                const s  = Math.min(sx, sy);
                const vx = (width  - s * dx) / 2 - s * minx;
                const vy = (height - s * dy) / 2 - s * miny;

                const pathColor = style === "light" ? "rgba(0,0,0,0.5)" : "rgba(255,255,255,0.5)"
                if (poly && poly.length >= 2) {
                    ctx.strokeStyle = pathColor;
                    ctx.lineWidth = 2;
                    ctx.beginPath();
                    for (let i = 0; i < poly.length; i++) {
                        const x = s * poly[i][0] + vx;
                        const y = s * poly[i][1] + vy;
                        if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
                    }
                    ctx.stroke();
                }

                if (marks && marks.length >= 1) {
                    ctx.fillStyle = styleAccentColor;
                    for (let i = 0; i < marks.length; i++) {
                        const x = s * marks[i][0] + vx;
                        const y = s * marks[i][1] + vy;
                        ctx.beginPath(); ctx.arc(x, y, 3, 0, 2*Math.PI); ctx.fill();
                    }
                }

                if (currPos && currPos.length >= 2) {
                    const x = s * currPos[0] + vx;
                    const y = s * currPos[1] + vy;
                    ctx.strokeStyle = styleAccentColor;
                    ctx.beginPath(); ctx.arc(x, y, 3, 0, 2*Math.PI); ctx.stroke();
                    ctx.beginPath(); ctx.arc(x, y, 4, 0, 2*Math.PI); ctx.stroke();
                }
            }
            Component.onCompleted: { refreshPolyline(); if (gpsDistanceMarksCheckbox.checked) refreshDistanceMarks(); }
        }
        Connections {
            target: controller
            function onChart_data_changed(): void {
                gpsSection.visible = controller.has_gps;
                if (gpsSection.visible && gpsMapCheckbox.checked) gpsMap.refreshPolyline();
            }
        }
    }

    FileDialog {
        id: exportFileDialog;
        fileMode: FileDialog.SaveFile;
        title: qsTr("Select file destination");
        type: "gyro-csv";
        property var exportData: ({});
        onAccepted: {
            if (exportData === "gpx") {
                controller.export_synchronized_gpx(selectedFile);
            } else if (exportData === "gps-distance-csv") {
                const step = gpsSection.getSafeNumberFieldValue(gpsDistanceStep, 10.0);
                const thr = gpsSection.getSafeNumberFieldValue(gpsSpeedThreshold, 0.0);
                controller.export_gps_distance_timestamps(selectedFile, step, thr);
            }
        }
    }

    Row {
        anchors.horizontalCenter: parent.horizontalCenter;
        LinkButton {
            id: exportGpsBtn;
            text: qsTr("Export");
            onClicked: menuLoader.toggle(exportGpsBtn, 0, height);
            Component {
                id: menu;
                Menu {
                    id: menuInner;
                    Action {
                        text: qsTr("Export synchronized GPX");
                        onTriggered: root.showExportDialog("gpx", "gpx");
                    }
                    Action {
                        text: qsTr("Export distance timestamps (CSV)");
                        onTriggered: root.showExportDialog("csv", "gps-distance-csv");
                    }
                }
            }
            ContextMenuLoader {
                id: menuLoader;
                sourceComponent: menu
            }
        }
    }
}
