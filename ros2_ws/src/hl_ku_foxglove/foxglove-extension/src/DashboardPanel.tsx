import { MessageEvent, PanelExtensionContext } from "@foxglove/extension";
import {
  ChangeEvent,
  ReactElement,
  useEffect,
  useLayoutEffect,
  useMemo,
  useRef,
  useState,
} from "react";
import { createRoot } from "react-dom/client";

type ViewName = "overview" | "perception" | "gnss" | "variants";
type Point2 = { x: number; y: number };
type VehiclePose = Point2 & { yaw: number };
type RosRecord = Record<string, unknown>;
type RawImage = {
  width: number;
  height: number;
  encoding: string;
  step: number;
  data: Uint8Array | readonly unknown[];
};
type Sample = {
  t: number;
  speed?: number;
  targetSpeed?: number;
  steering?: number;
  steeringTarget?: number;
  cte?: number;
  driveDuty?: number;
};
type MissionSegment = {
  zone: string;
  mission: string;
  direction: number;
  startIndex: number;
  endIndex: number;
  startSM: number;
  endSM: number;
  eventIds: string[];
};
type MissionEventPoint = Point2 & {
  eventId: string;
  index: number;
  sM: number;
};
type VariantMetadata = {
  slot: number;
  name: string;
  file: string;
  pointCount: number;
  segmentCount: number;
  segments: MissionSegment[];
  events: MissionEventPoint[];
};
type SourceCourseProgress = {
  course: string;
  segmentName: string;
  localS: number;
  localTotal: number;
  sourceS: number;
  sourceStart: number;
  sourceEnd: number;
  sourceTotal: number;
  sectionIndex: number;
  sectionStart: number;
  sectionEnd: number;
  expectedZone: string;
  expectedLabel: string;
};
type DashboardData = {
  nowSec: number;
  sourceMode: string;
  routeS?: number;
  routeLength?: number;
  cte?: number;
  zone: string;
  displayMission: string;
  actualMission: string;
  missionDetail: string;
  activeVariant: string;
  targetSpeed?: number;
  speed?: number;
  steering?: number;
  steeringTarget?: number;
  driveDuty?: number;
  driveEnabled?: boolean;
  brake?: boolean;
  faultFlags?: number;
  battery?: number;
  boardTemperature?: number;
  fixType?: number;
  headingValid?: boolean;
  positionValid?: boolean;
  satellites?: number;
  hdop?: number;
  correctionAge?: number;
  safety: string;
  recordedRoute: Point2[];
  activeRoute: Point2[];
  selectedRoute: Point2[];
  actualPath: Point2[];
  variants: Array<Point2[] | undefined>;
  variantMetadata: Array<VariantMetadata | undefined>;
  sourceCourseRoute: Point2[];
  sourceProgress?: SourceCourseProgress;
  currentPose?: VehiclePose;
  rawCamera?: RawImage;
  yoloCamera?: RawImage;
  history: Sample[];
};
type PanelConfig = {
  view?: ViewName;
  bagStartSec?: number;
  bagEndSec?: number;
};

const DEFAULT_BAG_START_SEC = 1788589641.6319368;
const DEFAULT_BAG_END_SEC = 1788590789.2265995;
const HISTORY_SECONDS = 35;
const VARIANT_COLORS = [
  "#ff595e",
  "#ff924c",
  "#ffca3a",
  "#8ac926",
  "#52b788",
  "#28a9ff",
  "#8f7cff",
  "#f15bb5",
];
const VARIANT_LABELS = [
  "T1 · P1 · END1",
  "T1 · P1 · END2",
  "T1 · P2 · END1",
  "T1 · P2 · END2",
  "T2 · P1 · END1",
  "T2 · P1 · END2",
  "T2 · P2 · END1",
  "T2 · P2 · END2",
];
const MISSION_ZONE_COLORS: Record<string, string> = {
  ROUTE: "#4b6470",
  HILL: "#f7d154",
  BEND: "#9aa6ff",
  TRAFFIC: "#c77dff",
  S_CURVE: "#ff8c61",
  T_PARK: "#28a9ff",
  DUMMY: "#ff5d8f",
  PARALLEL_PARK: "#16d7e9",
  END_LANE: "#ff9f43",
  FINISH: "#ff5d66",
};
const MISSION_ZONE_LABELS: Record<string, string> = {
  ROUTE: "일반",
  HILL: "언덕",
  BEND: "곡선",
  TRAFFIC: "신호",
  S_CURVE: "S자",
  T_PARK: "T주차",
  DUMMY: "돌발",
  PARALLEL_PARK: "평행주차",
  END_LANE: "종료차선",
  FINISH: "종료",
};

const TOPICS = [
  "/camera/image_raw",
  "/perception/yolo_overlay",
  "/visualization/recorded_route",
  "/visualization/active_route",
  "/visualization/source_course_route",
  "/planning/reference_path",
  "/visualization/actual_path",
  "/visualization/current_pose",
  "/localization/gnss_pose",
  "/visualization/replay_route_s",
  "/visualization/source_course_progress",
  "/visualization/cross_track_error_m",
  "/visualization/replay_zone",
  "/visualization/display_mission",
  "/visualization/active_variant",
  "/visualization/replay_mode",
  "/visualization/route_variant_metadata",
  "/visualization/target_speed_mps",
  "/mission/status",
  "/gnss/status",
  "/vehicle/feedback",
  "/safety/status",
  ...Array.from({ length: 8 }, (_, index) =>
    `/visualization/route_variant_${String(index + 1).padStart(2, "0")}`,
  ),
];

const FSM_DEFINITIONS = [
  { name: "RouteFSM", ko: "전역경로 주행", matches: ["NORMAL", "ROUTE", "READY", "INIT"] },
  { name: "HillFSM", ko: "언덕 정지·통과", matches: ["HILL"] },
  { name: "CurveFSM", ko: "곡선·S자 구간", matches: ["S_OBSTACLE", "S_CURVE", "BEND"] },
  { name: "TrafficFSM", ko: "신호 판단", matches: ["TRAFFIC"] },
  { name: "T-ParkingFSM", ko: "T자 주차", matches: ["PERP_PARK", "T_PARK"] },
  { name: "DummyFSM", ko: "돌발 장애물", matches: ["DUMMY"] },
  { name: "ParallelFSM", ko: "평행 주차", matches: ["PARALLEL_PARK"] },
  { name: "EndLaneFSM", ko: "종료 차선 변경", matches: ["END_LANE"] },
  { name: "FinishFSM", ko: "주행 종료", matches: ["FINISH"] },
] as const;

function asRecord(value: unknown): RosRecord | undefined {
  return typeof value === "object" && value != undefined
    ? (value as RosRecord)
    : undefined;
}

function numeric(record: RosRecord | undefined, key: string): number | undefined {
  const value = record?.[key];
  return typeof value === "number" && Number.isFinite(value) ? value : undefined;
}

function textual(record: RosRecord | undefined, key: string): string | undefined {
  const value = record?.[key];
  return typeof value === "string" ? value : undefined;
}

function booleanValue(record: RosRecord | undefined, key: string): boolean | undefined {
  const value = record?.[key];
  return typeof value === "boolean" ? value : undefined;
}

function parseVariantMetadata(payload: string): Array<VariantMetadata | undefined> {
  const result: Array<VariantMetadata | undefined> = Array.from({ length: 8 });
  try {
    const root = asRecord(JSON.parse(payload));
    const variants = root?.["variants"];
    if (!Array.isArray(variants)) {
      return result;
    }
    for (const value of variants) {
      const record = asRecord(value);
      const slot = numeric(record, "slot");
      const rawSegments = record?.["segments"];
      if (slot == undefined || slot < 1 || slot > 8 || !Array.isArray(rawSegments)) {
        continue;
      }
      const segments: MissionSegment[] = [];
      for (const rawSegment of rawSegments) {
        const segment = asRecord(rawSegment);
        const startIndex = numeric(segment, "start_index");
        const endIndex = numeric(segment, "end_index");
        const startSM = numeric(segment, "start_s_m");
        const endSM = numeric(segment, "end_s_m");
        if (startIndex == undefined || endIndex == undefined || startSM == undefined || endSM == undefined) {
          continue;
        }
        const rawEventIds = segment?.["event_ids"];
        segments.push({
          zone: textual(segment, "zone") ?? "ROUTE",
          mission: textual(segment, "mission") ?? "NORMAL",
          direction: numeric(segment, "direction") ?? 1,
          startIndex,
          endIndex,
          startSM,
          endSM,
          eventIds: Array.isArray(rawEventIds)
            ? rawEventIds.filter((eventId): eventId is string => typeof eventId === "string")
            : [],
        });
      }
      const events: MissionEventPoint[] = [];
      const rawEvents = record?.["events"];
      if (Array.isArray(rawEvents)) {
        for (const rawEvent of rawEvents) {
          const event = asRecord(rawEvent);
          const eventId = textual(event, "event_id");
          const index = numeric(event, "index");
          const sM = numeric(event, "s_m");
          const x = numeric(event, "x_m");
          const y = numeric(event, "y_m");
          if (eventId != undefined && index != undefined && sM != undefined && x != undefined && y != undefined) {
            events.push({ eventId, index, sM, x, y });
          }
        }
      }
      result[slot - 1] = {
        slot,
        name: textual(record, "name") ?? `variant_${String(slot).padStart(2, "0")}`,
        file: textual(record, "file") ?? "",
        pointCount: numeric(record, "point_count") ?? 0,
        segmentCount: numeric(record, "segment_count") ?? segments.length,
        segments,
        events,
      };
    }
  } catch {
    return result;
  }
  return result;
}

function parseSourceCourseProgress(payload: string): SourceCourseProgress | undefined {
  try {
    const record = asRecord(JSON.parse(payload));
    const localS = numeric(record, "local_s_m");
    const localTotal = numeric(record, "local_total_m");
    const sourceS = numeric(record, "source_s_m");
    const sourceStart = numeric(record, "source_start_s_m");
    const sourceEnd = numeric(record, "source_end_s_m");
    const sourceTotal = numeric(record, "source_total_m");
    const sectionIndex = numeric(record, "section_index");
    const sectionStart = numeric(record, "section_start_s_m");
    const sectionEnd = numeric(record, "section_end_s_m");
    if (
      localS == undefined || localTotal == undefined || sourceS == undefined ||
      sourceStart == undefined || sourceEnd == undefined || sourceTotal == undefined ||
      sectionIndex == undefined || sectionStart == undefined || sectionEnd == undefined
    ) {
      return undefined;
    }
    return {
      course: textual(record, "course") ?? "course_07",
      segmentName: textual(record, "segment_name") ?? "",
      localS,
      localTotal,
      sourceS,
      sourceStart,
      sourceEnd,
      sourceTotal,
      sectionIndex,
      sectionStart,
      sectionEnd,
      expectedZone: textual(record, "expected_zone") ?? "ROUTE",
      expectedLabel: textual(record, "expected_label") ?? "일반구간",
    };
  } catch {
    return undefined;
  }
}

function pathPoints(record: RosRecord | undefined): Point2[] {
  const poses = record?.["poses"];
  if (!Array.isArray(poses)) {
    return [];
  }
  const result: Point2[] = [];
  for (const stampedPose of poses) {
    const poseRecord = asRecord(stampedPose);
    const pose = asRecord(poseRecord?.["pose"]) ?? poseRecord;
    const position = asRecord(pose?.["position"]);
    const x = numeric(position, "x");
    const y = numeric(position, "y");
    if (x != undefined && y != undefined) {
      result.push({ x, y });
    }
  }
  return result;
}

function vehiclePose(record: RosRecord | undefined): VehiclePose | undefined {
  let pose = asRecord(record?.["pose"]);
  pose = asRecord(pose?.["pose"]) ?? pose;
  const position = asRecord(pose?.["position"]);
  const orientation = asRecord(pose?.["orientation"]);
  const x = numeric(position, "x");
  const y = numeric(position, "y");
  const qx = numeric(orientation, "x") ?? 0;
  const qy = numeric(orientation, "y") ?? 0;
  const qz = numeric(orientation, "z") ?? 0;
  const qw = numeric(orientation, "w") ?? 1;
  if (x == undefined || y == undefined) {
    return undefined;
  }
  const yaw = Math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz));
  return { x, y, yaw };
}

function rawImage(record: RosRecord | undefined): RawImage | undefined {
  const width = numeric(record, "width");
  const height = numeric(record, "height");
  const encoding = textual(record, "encoding");
  const data = record?.["data"];
  if (
    width == undefined ||
    height == undefined ||
    encoding == undefined ||
    (!(data instanceof Uint8Array) && !Array.isArray(data))
  ) {
    return undefined;
  }
  return {
    width,
    height,
    encoding: encoding.toLowerCase(),
    step: numeric(record, "step") ?? width * 3,
    data,
  };
}

function eventTimeSec(event: MessageEvent): number | undefined {
  const receiveTime = event.receiveTime;
  return receiveTime.sec + receiveTime.nsec / 1e9;
}

function blankData(): DashboardData {
  return {
    nowSec: DEFAULT_BAG_START_SEC,
    sourceMode: "ROSBAG REPLAY",
    zone: "WAITING",
    displayMission: "판단 데이터 대기",
    actualMission: "",
    missionDetail: "",
    activeVariant: "경로 선택 대기",
    safety: "NO DATA",
    recordedRoute: [],
    activeRoute: [],
    sourceCourseRoute: [],
    selectedRoute: [],
    actualPath: [],
    variants: Array.from({ length: 8 }),
    variantMetadata: Array.from({ length: 8 }),
    history: [],
  };
}

function formatDuration(seconds: number): string {
  const safe = Math.max(0, seconds);
  const hours = Math.floor(safe / 3600);
  const minutes = Math.floor((safe % 3600) / 60);
  const remainder = Math.floor(safe % 60);
  const millis = Math.floor((safe - Math.floor(safe)) * 1000);
  return `${String(hours).padStart(2, "0")}:${String(minutes).padStart(2, "0")}:${String(
    remainder,
  ).padStart(2, "0")}.${String(millis).padStart(3, "0")}`;
}

function formatMetric(value: number | undefined, digits = 2): string {
  return value == undefined ? "--" : value.toFixed(digits);
}

function messageString(record: RosRecord | undefined): string | undefined {
  return textual(record, "data");
}

function useDashboardData(context: PanelExtensionContext): DashboardData {
  const [data, setData] = useState<DashboardData>(blankData);
  const dataRef = useRef<DashboardData>(data);
  const lastSampleTime = useRef<number>(-Infinity);

  useLayoutEffect(() => {
    context.subscribe(TOPICS.map((topic) => ({ topic })));
    context.watch("currentFrame");
    context.watch("currentTime");

    context.onRender = (renderState, done) => {
      const previous = dataRef.current;
      const renderTime = renderState.currentTime;
      const nowSec =
        renderTime == undefined
          ? previous.nowSec
          : renderTime.sec + renderTime.nsec / 1e9;
      const next: DashboardData = {
        ...previous,
        nowSec,
        variants: [...previous.variants],
        variantMetadata: [...previous.variantMetadata],
      };

      if (nowSec < previous.nowSec - 0.5) {
        next.history = [];
        next.actualPath = [];
        lastSampleTime.current = -Infinity;
      }

      for (const event of renderState.currentFrame ?? []) {
        const message = asRecord(event.message);
        const topic = event.topic;
        if (topic === "/camera/image_raw") {
          next.rawCamera = rawImage(message) ?? next.rawCamera;
        } else if (topic === "/perception/yolo_overlay") {
          next.yoloCamera = rawImage(message) ?? next.yoloCamera;
        } else if (topic === "/visualization/recorded_route") {
          next.recordedRoute = pathPoints(message);
        } else if (topic === "/visualization/active_route") {
          next.activeRoute = pathPoints(message);
          next.routeLength = pathLength(next.activeRoute);
        } else if (topic === "/visualization/source_course_route") {
          next.sourceCourseRoute = pathPoints(message);
        } else if (topic === "/planning/reference_path") {
          next.selectedRoute = pathPoints(message);
        } else if (topic === "/visualization/actual_path") {
          next.actualPath = pathPoints(message);
        } else if (topic === "/visualization/current_pose") {
          next.currentPose = vehiclePose(message) ?? next.currentPose;
        } else if (topic === "/localization/gnss_pose") {
          const livePose = vehiclePose(message);
          if (livePose != undefined) {
            next.currentPose = livePose;
            const last = next.actualPath[next.actualPath.length - 1];
            if (last == undefined || Math.hypot(livePose.x - last.x, livePose.y - last.y) >= 0.15) {
              next.actualPath = [...next.actualPath, { x: livePose.x, y: livePose.y }].slice(-6000);
            }
          }
        } else if (topic === "/visualization/replay_route_s") {
          next.routeS = numeric(message, "data");
        } else if (topic === "/visualization/source_course_progress") {
          const payload = messageString(message);
          if (payload != undefined) {
            next.sourceProgress = parseSourceCourseProgress(payload) ?? next.sourceProgress;
          }
        } else if (topic === "/visualization/cross_track_error_m") {
          next.cte = numeric(message, "data");
        } else if (topic === "/visualization/replay_zone") {
          next.zone = messageString(message) ?? next.zone;
        } else if (topic === "/visualization/display_mission") {
          next.displayMission = messageString(message) ?? next.displayMission;
        } else if (topic === "/visualization/active_variant") {
          next.activeVariant = messageString(message) ?? next.activeVariant;
        } else if (topic === "/visualization/replay_mode") {
          const mode = messageString(message);
          if (mode != undefined) {
            next.sourceMode = mode.includes("LIVE_DRIVE") ? "LIVE DRIVE" : "ROSBAG REPLAY";
          }
        } else if (topic === "/visualization/route_variant_metadata") {
          const payload = messageString(message);
          if (payload != undefined) {
            next.variantMetadata = parseVariantMetadata(payload);
          }
        } else if (topic === "/visualization/target_speed_mps") {
          next.targetSpeed = numeric(message, "data");
        } else if (topic === "/mission/status") {
          next.actualMission = textual(message, "state_name") ?? "";
          next.missionDetail = textual(message, "detail") ?? "";
          next.zone = textual(message, "zone") ?? next.zone;
          next.targetSpeed = numeric(message, "speed_limit_mps") ?? next.targetSpeed;
        } else if (topic === "/gnss/status") {
          next.speed = numeric(message, "ground_speed_mps");
          next.fixType = numeric(message, "fix_type");
          next.headingValid = booleanValue(message, "heading_valid");
          next.positionValid = booleanValue(message, "position_valid");
          next.satellites = numeric(message, "satellites");
          next.hdop = numeric(message, "hdop");
          next.correctionAge = numeric(message, "correction_age_sec");
        } else if (topic === "/vehicle/feedback") {
          next.steering = numeric(message, "steering_angle_rad");
          next.steeringTarget = numeric(message, "steering_target_rad");
          next.driveDuty = numeric(message, "applied_drive_duty");
          next.driveEnabled = booleanValue(message, "drive_enabled");
          next.brake = booleanValue(message, "brake_active");
          next.faultFlags = numeric(message, "fault_flags");
          next.battery = numeric(message, "battery_voltage");
          next.boardTemperature = numeric(message, "board_temperature_c");
        } else if (topic === "/safety/status") {
          next.safety = messageString(message) ?? next.safety;
        } else {
          const match = /\/visualization\/route_variant_(\d{2})/.exec(topic);
          if (match?.[1] != undefined) {
            const index = Number(match[1]) - 1;
            if (index >= 0 && index < 8) {
              next.variants[index] = pathPoints(message);
            }
          }
        }

        const stamp = eventTimeSec(event);
        if (stamp != undefined && stamp > next.nowSec) {
          next.nowSec = stamp;
        }
      }

      if (next.nowSec - lastSampleTime.current >= 0.18) {
        const cutoff = next.nowSec - HISTORY_SECONDS;
        next.history = [
          ...next.history.filter((sample) => sample.t >= cutoff),
          {
            t: next.nowSec,
            speed: next.speed,
            targetSpeed: next.targetSpeed,
            steering: next.steering,
            steeringTarget: next.steeringTarget,
            cte: next.cte,
            driveDuty: next.driveDuty,
          },
        ].slice(-240);
        lastSampleTime.current = next.nowSec;
      }

      dataRef.current = next;
      setData(next);
      done();
    };

    return () => {
      context.onRender = undefined;
      context.subscribe([]);
    };
  }, [context]);

  return data;
}

function CameraView({ image, title, waiting }: { image?: RawImage; title: string; waiting: string }): ReactElement {
  const canvasRef = useRef<HTMLCanvasElement>(null);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (canvas == undefined || image == undefined) {
      return;
    }
    const source =
      image.data instanceof Uint8Array
        ? image.data
        : new Uint8Array(image.data.map((value) => Number(value)));
    const rgba = new Uint8ClampedArray(image.width * image.height * 4);
    const channels = image.encoding === "mono8" ? 1 : image.encoding.includes("rgba") ? 4 : 3;
    const bgr = image.encoding.startsWith("bgr") || image.encoding.startsWith("bgra");
    for (let y = 0; y < image.height; y += 1) {
      for (let x = 0; x < image.width; x += 1) {
        const sourceIndex = y * image.step + x * channels;
        const targetIndex = (y * image.width + x) * 4;
        if (channels === 1) {
          const value = source[sourceIndex] ?? 0;
          rgba[targetIndex] = value;
          rgba[targetIndex + 1] = value;
          rgba[targetIndex + 2] = value;
        } else {
          const first = source[sourceIndex] ?? 0;
          const second = source[sourceIndex + 1] ?? 0;
          const third = source[sourceIndex + 2] ?? 0;
          rgba[targetIndex] = bgr ? third : first;
          rgba[targetIndex + 1] = second;
          rgba[targetIndex + 2] = bgr ? first : third;
        }
        rgba[targetIndex + 3] = channels === 4 ? (source[sourceIndex + 3] ?? 255) : 255;
      }
    }
    canvas.width = image.width;
    canvas.height = image.height;
    canvas.getContext("2d")?.putImageData(new ImageData(rgba, image.width, image.height), 0, 0);
  }, [image]);

  return (
    <section className="panel camera-panel">
      <div className="section-title">{title}</div>
      {image == undefined ? (
        <div className="waiting">{waiting}</div>
      ) : (
        <canvas ref={canvasRef} className="camera-canvas" />
      )}
    </section>
  );
}

function pointsAttribute(points: Point2[]): string {
  return points.map((point) => `${point.x.toFixed(2)},${(-point.y).toFixed(2)}`).join(" ");
}

function pathLength(points: Point2[]): number | undefined {
  if (points.length < 2) {
    return undefined;
  }
  let length = 0;
  for (let index = 1; index < points.length; index += 1) {
    const previous = points[index - 1];
    const current = points[index];
    if (previous != undefined && current != undefined) {
      length += Math.hypot(current.x - previous.x, current.y - previous.y);
    }
  }
  return length;
}

function pathPointAtDistance(points: Point2[], distance: number): Point2 | undefined {
  if (points.length === 0) {
    return undefined;
  }
  let remaining = Math.max(0, distance);
  for (let index = 1; index < points.length; index += 1) {
    const first = points[index - 1];
    const second = points[index];
    if (first == undefined || second == undefined) {
      continue;
    }
    const length = Math.hypot(second.x - first.x, second.y - first.y);
    if (remaining <= length && length > 0) {
      const ratio = remaining / length;
      return {
        x: first.x + ratio * (second.x - first.x),
        y: first.y + ratio * (second.y - first.y),
      };
    }
    remaining -= length;
  }
  return points.at(-1);
}

function pathSliceByDistance(points: Point2[], start: number, end: number): Point2[] {
  if (points.length < 2 || end <= start) {
    return [];
  }
  const result: Point2[] = [];
  const firstPoint = pathPointAtDistance(points, start);
  if (firstPoint != undefined) {
    result.push(firstPoint);
  }
  let travelled = 0;
  for (let index = 1; index < points.length; index += 1) {
    const first = points[index - 1];
    const second = points[index];
    if (first == undefined || second == undefined) {
      continue;
    }
    travelled += Math.hypot(second.x - first.x, second.y - first.y);
    if (travelled > start && travelled < end) {
      result.push(second);
    }
  }
  const lastPoint = pathPointAtDistance(points, end);
  if (lastPoint != undefined) {
    result.push(lastPoint);
  }
  return result;
}

function SourceCourseInset({ data }: { data: DashboardData }): ReactElement | null {
  const progress = data.sourceProgress;
  const route = data.sourceCourseRoute;
  const geometry = useMemo(() => {
    if (progress == undefined || route.length < 2) {
      return undefined;
    }
    const totalPathLength = pathLength(route) ?? progress.sourceTotal;
    const scale = totalPathLength / Math.max(progress.sourceTotal, 0.001);
    const marker = pathPointAtDistance(route, progress.sourceS * scale);
    const selected = pathSliceByDistance(
      route,
      progress.sourceStart * scale,
      progress.sourceEnd * scale,
    );
    const xs = route.map((point) => point.x);
    const ys = route.map((point) => point.y);
    const width = Math.max(1, Math.max(...xs) - Math.min(...xs));
    const height = Math.max(1, Math.max(...ys) - Math.min(...ys));
    const margin = Math.max(width, height) * 0.05;
    return {
      marker,
      selected,
      viewBox: `${Math.min(...xs) - margin} ${-(Math.max(...ys) + margin)} ${width + 2 * margin} ${height + 2 * margin}`,
    };
  }, [progress, route]);
  if (progress == undefined || geometry == undefined) {
    return null;
  }
  const percent = Math.max(0, Math.min(100, progress.sourceS / Math.max(progress.sourceTotal, 0.001) * 100));
  return (
    <aside className="source-course-inset">
      <div className="source-course-heading">
        <span>시험장 원본 위치</span>
        <b>{progress.expectedZone} · {progress.expectedLabel}</b>
      </div>
      <svg viewBox={geometry.viewBox} preserveAspectRatio="xMidYMid meet">
        <polyline points={pointsAttribute(route)} fill="none" stroke="#506772" strokeWidth="0.35" vectorEffect="non-scaling-stroke" />
        {geometry.selected.length > 1 && <polyline points={pointsAttribute(geometry.selected)} fill="none" stroke="#16d7e9" strokeWidth="0.8" vectorEffect="non-scaling-stroke" />}
        {geometry.marker != undefined && <circle cx={geometry.marker.x} cy={-geometry.marker.y} r="1.2" fill="#f7d154" stroke="#ffffff" strokeWidth="0.3" vectorEffect="non-scaling-stroke" />}
      </svg>
      <div className="source-progress-track"><i style={{ width: `${percent}%` }} /></div>
      <strong>원본 s {progress.sourceS.toFixed(1)} / {progress.sourceTotal.toFixed(1)} m</strong>
      <span>선택 {progress.sourceStart.toFixed(1)}–{progress.sourceEnd.toFixed(1)} m · 로컬 {progress.localS.toFixed(1)} m</span>
    </aside>
  );
}

function RouteMap({
  data,
  local = false,
  showVariants = false,
  selectedVariantIndex = 0,
  onSelectVariant,
}: {
  data: DashboardData;
  local?: boolean;
  showVariants?: boolean;
  selectedVariantIndex?: number;
  onSelectVariant?: (index: number) => void;
}): ReactElement {
  const bounds = useMemo(() => {
    if (local && data.currentPose != undefined) {
      return {
        minX: data.currentPose.x - 12,
        maxX: data.currentPose.x + 12,
        minY: data.currentPose.y - 12,
        maxY: data.currentPose.y + 12,
      };
    }
    const paths = [
      data.recordedRoute,
      data.activeRoute,
      data.selectedRoute,
      data.actualPath,
      ...(showVariants ? data.variants.filter((path): path is Point2[] => path != undefined) : []),
    ];
    const all = paths.flat();
    if (all.length === 0) {
      return { minX: -10, maxX: 10, minY: -10, maxY: 10 };
    }
    const xs = all.map((point) => point.x);
    const ys = all.map((point) => point.y);
    const margin = 5;
    return {
      minX: Math.min(...xs) - margin,
      maxX: Math.max(...xs) + margin,
      minY: Math.min(...ys) - margin,
      maxY: Math.max(...ys) + margin,
    };
  }, [data.activeRoute, data.actualPath, data.currentPose, data.recordedRoute, data.selectedRoute, data.variants, local, showVariants]);

  const width = Math.max(1, bounds.maxX - bounds.minX);
  const height = Math.max(1, bounds.maxY - bounds.minY);
  const viewBox = `${bounds.minX} ${-bounds.maxY} ${width} ${height}`;
  const vehicle = data.currentPose;
  const selectedVariantPath = data.variants[selectedVariantIndex];
  const selectedVariantMetadata = data.variantMetadata[selectedVariantIndex];
  const hillStop = selectedVariantMetadata?.events.find((event) => event.eventId === "HILL_STOP");
  const selectedVariantLabel = VARIANT_LABELS[selectedVariantIndex] ?? "경로 선택 대기";
  const markedRoute = data.activeRoute.length > 1 ? data.activeRoute : data.selectedRoute;
  const routeStart = markedRoute[0];
  const routeEnd = markedRoute[markedRoute.length - 1];
  const liveRoute = data.sourceMode === "LIVE DRIVE";

  return (
    <section className="panel map-panel">
      <div className="map-heading">
        <div>
          <div className="section-title">{local ? "LOCAL ROUTE TRACKING" : showVariants ? "8 ROUTE VARIANTS" : liveRoute ? "현재 시험 구간 · 기준경로 / 실주행" : "ROUTE OVERVIEW"}</div>
          <div className="section-subtitle">{showVariants ? `${selectedVariantLabel} · ${selectedVariantMetadata?.segmentCount ?? "--"} waypoint 구간 · 미션색 표시` : `${data.activeVariant} · 전체 ${formatMetric(data.routeLength, 2)} m`}</div>
          {!showVariants && <div className="section-subtitle map-live-metrics">건대 진행 {formatMetric(data.routeS, 2)} m · 시험장 원본 {formatMetric(data.sourceProgress?.sourceS, 1)} / {formatMetric(data.sourceProgress?.sourceTotal, 1)} m · CTE {formatMetric(data.cte)} m</div>}
        </div>
        <span className="source-badge">{showVariants ? `SELECTED ${String(selectedVariantIndex + 1).padStart(2, "0")}` : data.actualMission ? "ACTUAL" : "INFERRED"}</span>
      </div>
      {showVariants && onSelectVariant != undefined && (
        <div className="compact-variant-selector">
          {data.variants.map((path, index) => (
            <button type="button" key={index} disabled={path == undefined} className={selectedVariantIndex === index ? "selected" : ""} onClick={() => { onSelectVariant(index); }}>
              {String(index + 1).padStart(2, "0")}
            </button>
          ))}
        </div>
      )}
      <svg className="route-svg" viewBox={viewBox} preserveAspectRatio="xMidYMid meet">
        <defs>
          <pattern id={local ? "local-grid" : showVariants ? "variant-grid" : "overview-grid"} width={width / 10} height={height / 10} patternUnits="userSpaceOnUse">
            <path d={`M ${width / 10} 0 L 0 0 0 ${height / 10}`} fill="none" stroke="#1d3444" strokeWidth={Math.max(width, height) / 900} />
          </pattern>
        </defs>
        <rect x={bounds.minX} y={-bounds.maxY} width={width} height={height} fill={`url(#${local ? "local-grid" : showVariants ? "variant-grid" : "overview-grid"})`} />
        {!showVariants && data.recordedRoute.length > 1 && <polyline points={pointsAttribute(data.recordedRoute)} fill="none" stroke="#728492" strokeWidth={local ? 0.12 : 0.22} opacity="0.55" vectorEffect="non-scaling-stroke" />}
        {!showVariants && data.activeRoute.length > 1 && <polyline points={pointsAttribute(data.activeRoute)} fill="none" stroke="#16d7e9" strokeWidth={local ? 0.18 : 0.32} vectorEffect="non-scaling-stroke" />}
        {showVariants && data.variants.map((path, index) => index !== selectedVariantIndex && path != undefined && path.length > 1 ? <polyline key={index} points={pointsAttribute(path)} fill="none" stroke={VARIANT_COLORS[index]} strokeWidth="0.18" opacity="0.22" vectorEffect="non-scaling-stroke" /> : undefined)}
        {showVariants && selectedVariantPath != undefined && selectedVariantPath.length > 1 && selectedVariantMetadata == undefined && <polyline points={pointsAttribute(selectedVariantPath)} fill="none" stroke={VARIANT_COLORS[selectedVariantIndex]} strokeWidth="0.48" vectorEffect="non-scaling-stroke" />}
        {showVariants && selectedVariantPath != undefined && selectedVariantMetadata?.segments.map((segment, index) => {
          const start = Math.max(0, segment.startIndex - (segment.startIndex > 0 ? 1 : 0));
          const points = selectedVariantPath.slice(start, segment.endIndex + 1);
          const boundary = selectedVariantPath[segment.startIndex];
          if (points.length < 2) {
            return undefined;
          }
          return (
            <g key={`${segment.startIndex}-${segment.zone}`}>
              <polyline points={pointsAttribute(points)} fill="none" stroke={MISSION_ZONE_COLORS[segment.zone] ?? "#a6b8c0"} strokeWidth="0.55" opacity="1" vectorEffect="non-scaling-stroke" />
              {index > 0 && boundary != undefined && <circle cx={boundary.x} cy={-boundary.y} r="0.28" fill={MISSION_ZONE_COLORS[segment.zone] ?? "#a6b8c0"} stroke="#f5f8fa" strokeWidth="0.08" vectorEffect="non-scaling-stroke" />}
            </g>
          );
        })}
        {showVariants && hillStop != undefined && (
          <g className="hill-stop-marker">
            <circle cx={hillStop.x} cy={-hillStop.y} r="0.95" fill="#ff5d66" fillOpacity="0.28" stroke="#ffffff" strokeWidth="0.14" vectorEffect="non-scaling-stroke" />
            <circle cx={hillStop.x} cy={-hillStop.y} r="0.48" fill="#ff5d66" stroke="#f7d154" strokeWidth="0.16" vectorEffect="non-scaling-stroke" />
            <line x1={hillStop.x - 0.72} x2={hillStop.x + 0.72} y1={-hillStop.y} y2={-hillStop.y} stroke="#ffffff" strokeWidth="0.10" vectorEffect="non-scaling-stroke" />
            <line x1={hillStop.x} x2={hillStop.x} y1={-hillStop.y - 0.72} y2={-hillStop.y + 0.72} stroke="#ffffff" strokeWidth="0.10" vectorEffect="non-scaling-stroke" />
            <text x={hillStop.x + 1.15} y={-hillStop.y - 0.65} fill="#ffffff" fontSize="1.25" fontWeight="700" stroke="#071116" strokeWidth="0.08" paintOrder="stroke">HILL STOP · 3.2 s</text>
          </g>
        )}
        {!showVariants && data.selectedRoute.length > 1 && <polyline points={pointsAttribute(data.selectedRoute)} fill="none" stroke="#ff5d66" strokeWidth={local ? 0.22 : 0.38} vectorEffect="non-scaling-stroke" />}
        {!showVariants && data.actualPath.length > 1 && <polyline points={pointsAttribute(data.actualPath)} fill="none" stroke="#f5f8fa" strokeWidth={local ? 0.18 : 0.32} vectorEffect="non-scaling-stroke" />}
        {!showVariants && routeStart != undefined && <circle cx={routeStart.x} cy={-routeStart.y} r="0.28" fill="#31d59b" stroke="#ffffff" strokeWidth="0.08" vectorEffect="non-scaling-stroke" />}
        {!showVariants && routeEnd != undefined && <circle cx={routeEnd.x} cy={-routeEnd.y} r="0.28" fill="#ff5d66" stroke="#ffffff" strokeWidth="0.08" vectorEffect="non-scaling-stroke" />}
        {vehicle != undefined && (
          <g transform={`translate(${vehicle.x} ${-vehicle.y}) rotate(${(-vehicle.yaw * 180) / Math.PI})`}>
            <polygon points="1.1,0 -0.8,-0.55 -0.55,0 -0.8,0.55" fill="#f7d154" stroke="#ffffff" strokeWidth="0.08" vectorEffect="non-scaling-stroke" />
          </g>
        )}
      </svg>
      {liveRoute && <SourceCourseInset data={data} />}
      {showVariants ? (
        <div className="map-legend mission-map-legend">
          <span><i className="hill-stop-dot" />언덕 정지점</span>
          {Object.entries(MISSION_ZONE_LABELS).map(([zone, label]) => <span key={zone}><i style={{ background: MISSION_ZONE_COLORS[zone] }} />{label}</span>)}
        </div>
      ) : (
        <div className="map-legend">
          <span><i style={{ background: "#16d7e9" }} />기준 경로</span>
          <span><i style={{ background: "#ff5d66" }} />선택/계획</span>
          <span><i style={{ background: "#f5f8fa" }} />실제 궤적</span>
          <span><i className="start-dot" />시작</span>
          <span><i className="end-dot" />종료</span>
        </div>
      )}
    </section>
  );
}

function FsmGrid({ data }: { data: DashboardData }): ReactElement {
  const expectedState = data.sourceProgress?.expectedZone;
  const selectedState = expectedState ?? (data.actualMission.length > 0 ? data.actualMission : data.zone);
  const matchingState = (expectedState ?? `${data.actualMission} ${data.zone}`).toUpperCase();
  const inferred = expectedState != undefined || data.actualMission.length === 0;
  return (
    <section className="panel fsm-panel">
      <div className="fsm-header">
        <div>
          <div className="section-title">행동 선택 · MISSION FSM</div>
          <div className="section-subtitle">활성 카드=시험장 원본 구간 · 아래에 실제 제어 FSM을 함께 표시합니다.</div>
        </div>
        <div className="selected-state">
          <small>SELECTED STATE</small>
          <strong>{selectedState}</strong>
          <span>{expectedState != undefined ? `원본 예정 · 실제 ${data.actualMission || data.zone}` : inferred ? "경로 태그 추정" : "실제 /mission/status"}</span>
        </div>
      </div>
      <div className="fsm-grid">
        {FSM_DEFINITIONS.map((fsm, index) => {
          const active = fsm.matches.some((token) => matchingState.includes(token));
          return (
            <article key={fsm.name} className={`fsm-card ${active ? "active" : ""}`}>
              <span className="fsm-index">{String(index + 1).padStart(2, "0")}</span>
              <strong>{fsm.name}</strong>
              <b>{active ? selectedState : "INACTIVE"}</b>
              <span>{active ? data.missionDetail || fsm.ko : "판단 대기"}</span>
              <em>{active ? (expectedState != undefined ? "COURSE EXPECTED" : inferred ? "INFERRED" : "ACTIVE") : "STANDBY"}</em>
            </article>
          );
        })}
      </div>
    </section>
  );
}

type SeriesDefinition = {
  key: keyof Omit<Sample, "t">;
  label: string;
  color: string;
};

function SignalChart({
  title,
  data,
  series,
  minimum,
  maximum,
  unit,
}: {
  title: string;
  data: DashboardData;
  series: SeriesDefinition[];
  minimum: number;
  maximum: number;
  unit: string;
}): ReactElement {
  const samples = data.history;
  const minTime = samples.length > 0 ? (samples[0]?.t ?? data.nowSec - HISTORY_SECONDS) : data.nowSec - HISTORY_SECONDS;
  const maxTime = Math.max(data.nowSec, minTime + 1);
  const plotLeft = 48;
  const plotRight = 990;
  const plotTop = 30;
  const plotBottom = 150;
  const x = (time: number): number => plotLeft + ((time - minTime) / (maxTime - minTime)) * (plotRight - plotLeft);
  const y = (value: number): number => plotBottom - ((value - minimum) / (maximum - minimum)) * (plotBottom - plotTop);

  return (
    <section className="panel chart-panel">
      <div className="chart-heading">
        <div className="section-title">{title}</div>
        <div className="chart-values">
          {series.map((item) => {
            const latest = samples.at(-1)?.[item.key];
            return <span key={item.key} style={{ color: item.color }}>{item.label} {formatMetric(latest)} {unit}</span>;
          })}
        </div>
      </div>
      <svg viewBox="0 0 1000 170" className="chart-svg" preserveAspectRatio="none">
        {[0, 0.25, 0.5, 0.75, 1].map((ratio) => {
          const lineY = plotTop + ratio * (plotBottom - plotTop);
          const value = maximum - ratio * (maximum - minimum);
          return (
            <g key={ratio}>
              <line x1={plotLeft} x2={plotRight} y1={lineY} y2={lineY} stroke="#243642" strokeWidth="1" />
              <text x="4" y={lineY + 4} fill="#7f929f" fontSize="11">{value.toFixed(1)}</text>
            </g>
          );
        })}
        {series.map((item) => {
          const points = samples
            .filter((sample) => typeof sample[item.key] === "number")
            .map((sample) => `${x(sample.t).toFixed(1)},${y(sample[item.key]!).toFixed(1)}`)
            .join(" ");
          return points.length > 0 ? <polyline key={item.key} points={points} fill="none" stroke={item.color} strokeWidth="2.5" vectorEffect="non-scaling-stroke" /> : undefined;
        })}
      </svg>
    </section>
  );
}

function MetricCard({ label, value, status = "neutral" }: { label: string; value: string; status?: "good" | "warn" | "bad" | "neutral" }): ReactElement {
  return (
    <article className={`metric-card ${status}`}>
      <span>{label}</span>
      <strong>{value}</strong>
    </article>
  );
}

function VehicleSummary({ data }: { data: DashboardData }): ReactElement {
  const fixLabel = data.fixType === 4 ? "RTK FIXED" : data.fixType === 5 ? "RTK FLOAT" : data.fixType == undefined ? "NO DATA" : `FIX ${data.fixType}`;
  return (
    <section className="panel summary-panel">
      <div className="mission-summary">
        <small>SELECTED BEHAVIOR / FSM STATE</small>
        <strong>{data.actualMission || data.zone}</strong>
        <span>{data.actualMission ? data.missionDetail || "실제 미션 상태" : "경로 태그 기반 추정"}</span>
      </div>
      <div className="summary-metrics">
        <MetricCard label="시험장 원본 위치" value={data.sourceProgress == undefined ? "--" : `${data.sourceProgress.sourceS.toFixed(1)} / ${data.sourceProgress.sourceTotal.toFixed(1)} m`} />
        <MetricCard label="원본 예상 FSM" value={data.sourceProgress?.expectedZone ?? "--"} status={data.sourceProgress == undefined ? "neutral" : "good"} />
        <MetricCard label="실제 제어 FSM" value={data.actualMission || data.zone || "--"} />
        <MetricCard label="목표 속도" value={`${formatMetric(data.targetSpeed)} m/s`} />
        <MetricCard label="실제 속도" value={`${formatMetric(data.speed)} m/s`} />
        <MetricCard label="목표 조향" value={`${formatMetric(data.steeringTarget)} rad`} />
        <MetricCard label="실제 조향" value={`${formatMetric(data.steering)} rad`} />
        <MetricCard label="GNSS" value={fixLabel} status={data.fixType === 4 ? "good" : "warn"} />
        <MetricCard label="BRAKE" value={data.brake == undefined ? "--" : data.brake ? "ON" : "OFF"} status={data.brake === true ? "bad" : data.brake === false ? "good" : "neutral"} />
      </div>
    </section>
  );
}

function Timeline({ context, data, startSec, endSec }: { context: PanelExtensionContext; data: DashboardData; startSec: number; endSec: number }): ReactElement {
  const duration = Math.max(0.1, endSec - startSec);
  const elapsed = Math.max(0, Math.min(duration, data.nowSec - startSec));
  const [dragValue, setDragValue] = useState<number | undefined>();
  const [paused, setPaused] = useState(false);
  const [rate, setRateValue] = useState(1);
  const [busy, setBusy] = useState(false);

  useEffect(() => {
    let active = true;
    const poll = (): void => {
      if (context.callService == undefined) {
        return;
      }
      void context.callService("/rosbag2_player/is_paused", {}).then((response) => {
        if (active) {
          setPaused(booleanValue(asRecord(response), "paused") ?? false);
        }
      }).catch(() => undefined);
    };
    poll();
    const timer = window.setInterval(poll, 1000);
    return () => {
      active = false;
      window.clearInterval(timer);
    };
  }, [context]);

  const seekTo = async (targetElapsed: number): Promise<void> => {
    const bounded = Math.max(0, Math.min(duration, targetElapsed));
    const absolute = startSec + bounded;
    const sec = Math.floor(absolute);
    const nanosec = Math.max(0, Math.min(999_999_999, Math.round((absolute - sec) * 1e9)));
    setBusy(true);
    try {
      if (context.callService != undefined) {
        await context.callService("/rosbag2_player/seek", { time: { sec, nanosec } });
      } else {
        context.seekPlayback?.({ sec, nsec: nanosec });
      }
    } finally {
      setDragValue(undefined);
      setBusy(false);
    }
  };

  const togglePause = async (): Promise<void> => {
    if (context.callService == undefined) {
      return;
    }
    setBusy(true);
    try {
      await context.callService("/rosbag2_player/toggle_paused", {});
      setPaused((value) => !value);
    } finally {
      setBusy(false);
    }
  };

  const changeRate = async (nextRate: number): Promise<void> => {
    setRateValue(nextRate);
    if (context.callService != undefined) {
      await context.callService("/rosbag2_player/set_rate", { rate: nextRate });
    }
  };

  const sliderValue = dragValue ?? elapsed;
  const onSliderChange = (event: ChangeEvent<HTMLInputElement>): void => {
    setDragValue(Number(event.currentTarget.value));
  };

  return (
    <footer className="timeline">
      <div className="timeline-controls">
        <button disabled={busy} onClick={() => { void seekTo(elapsed - 10); }}>−10s</button>
        <button className="play-button" disabled={busy} onClick={() => { void togglePause(); }}>{paused ? "▶" : "Ⅱ"}</button>
        <button disabled={busy} onClick={() => { void seekTo(elapsed + 10); }}>+10s</button>
      </div>
      <span className="timeline-time">{formatDuration(sliderValue)}</span>
      <input
        aria-label="rosbag replay position"
        type="range"
        min="0"
        max={duration}
        step="0.1"
        value={sliderValue}
        onChange={onSliderChange}
        onPointerUp={(event) => { void seekTo(Number(event.currentTarget.value)); }}
        onKeyUp={(event) => { void seekTo(Number(event.currentTarget.value)); }}
      />
      <span className="timeline-time">{formatDuration(duration)}</span>
      <div className="rate-controls">
        {[0.5, 1, 2].map((value) => <button key={value} className={rate === value ? "selected" : ""} onClick={() => { void changeRate(value); }}>{value}×</button>)}
      </div>
    </footer>
  );
}

function Overview({ data }: { data: DashboardData }): ReactElement {
  const [selectedVariantIndex, setSelectedVariantIndex] = useState(0);
  const live = data.sourceMode === "LIVE DRIVE";
  return (
    <main className="dashboard-grid">
      <div className="left-stack">
        <div className="overview-variant-map">
          <RouteMap data={data} showVariants={!live} selectedVariantIndex={selectedVariantIndex} onSelectVariant={setSelectedVariantIndex} />
        </div>
        <FsmGrid data={data} />
      </div>
      <div className="right-stack">
        <div className="camera-grid">
          <CameraView image={data.rawCamera} title="CAMERA · RAW" waiting="/camera/image_raw 대기" />
          <CameraView image={data.yoloCamera ?? data.rawCamera} title={data.yoloCamera ? "CAMERA · YOLO MISSION" : "CAMERA · RAW FALLBACK (YOLO 미연결)"} waiting="/camera/image_raw 대기" />
        </div>
        <VehicleSummary data={data} />
        <SignalChart title="속도 · TARGET / ACTUAL" data={data} series={[{ key: "targetSpeed", label: "목표", color: "#16d7e9" }, { key: "speed", label: "실제", color: "#f5f8fa" }]} minimum={0} maximum={1.2} unit="m/s" />
        <SignalChart title="조향각 · TARGET / ACTUAL" data={data} series={[{ key: "steeringTarget", label: "목표", color: "#f7d154" }, { key: "steering", label: "실제", color: "#f5f8fa" }]} minimum={-0.55} maximum={0.55} unit="rad" />
        <SignalChart title="경로 횡오차 · CTE" data={data} series={[{ key: "cte", label: "CTE", color: "#ff5d66" }]} minimum={-1} maximum={1} unit="m" />
      </div>
    </main>
  );
}

function PerceptionView({ data }: { data: DashboardData }): ReactElement {
  return (
    <main className="detail-view">
      <div className="detail-camera-grid">
        <CameraView image={data.rawCamera} title="카메라 원본" waiting="카메라 토픽 대기" />
        <CameraView image={data.yoloCamera ?? data.rawCamera} title={data.yoloCamera ? "YOLO 미션 인지" : "RAW 카메라 대체화면 · YOLO 미연결"} waiting="카메라 토픽 대기" />
      </div>
      <div className="perception-bottom">
        <RouteMap data={data} local />
        <FsmGrid data={data} />
        <section className="panel readiness-panel">
          <div className="section-title">인지 연결 상태</div>
          <MetricCard label="RAW CAMERA" value={data.rawCamera ? "ONLINE" : "WAITING"} status={data.rawCamera ? "good" : "warn"} />
          <MetricCard label="YOLO OVERLAY" value={data.yoloCamera ? "ONLINE" : "NOT CONNECTED"} status={data.yoloCamera ? "good" : "warn"} />
          <MetricCard label="현재 미션" value={data.actualMission || data.zone} />
          <p>차선·도로 경계 인식과 카메라 캘리브레이션은 현재 범위에서 제외했습니다.</p>
        </section>
      </div>
    </main>
  );
}

function GnssView({ data }: { data: DashboardData }): ReactElement {
  const fixGood = data.fixType === 4;
  return (
    <main className="gnss-view">
      <section className="status-card-grid">
        <MetricCard label="GNSS FIX" value={data.fixType === 4 ? "RTK FIXED" : data.fixType === 5 ? "RTK FLOAT" : `FIX ${data.fixType ?? "--"}`} status={fixGood ? "good" : "warn"} />
        <MetricCard label="HEADING" value={data.headingValid == undefined ? "--" : data.headingValid ? "VALID" : "INVALID"} status={data.headingValid === true ? "good" : data.headingValid === false ? "bad" : "neutral"} />
        <MetricCard label="SATELLITES" value={formatMetric(data.satellites, 0)} status={(data.satellites ?? 0) >= 15 ? "good" : "warn"} />
        <MetricCard label="HDOP" value={formatMetric(data.hdop)} status={(data.hdop ?? 99) <= 1.5 ? "good" : "warn"} />
        <MetricCard label="RTCM AGE" value={`${formatMetric(data.correctionAge)} s`} status={(data.correctionAge ?? 99) <= 2 ? "good" : "warn"} />
        <MetricCard label="FAULT FLAGS" value={String(data.faultFlags ?? "--")} status={(data.faultFlags ?? 0) === 0 ? "good" : "bad"} />
        <MetricCard label="BATTERY" value={`${formatMetric(data.battery)} V`} />
        <MetricCard label="BOARD TEMP" value={`${formatMetric(data.boardTemperature, 1)} °C`} />
      </section>
      <div className="gnss-main-grid">
        <RouteMap data={data} />
        <section className="panel safety-panel">
          <div className="section-title">기록된 SAFETY 상태</div>
          <strong>{data.safety}</strong>
          <div className="safety-row"><span>DRIVE ENABLED</span><b>{data.driveEnabled == undefined ? "--" : data.driveEnabled ? "YES" : "NO"}</b></div>
          <div className="safety-row"><span>BRAKE</span><b className={data.brake === true ? "bad-text" : "good-text"}>{data.brake == undefined ? "--" : data.brake ? "ON" : "OFF"}</b></div>
          <div className="safety-row"><span>DRIVE DUTY</span><b>{formatMetric(data.driveDuty)}</b></div>
          <p>표시값은 rosbag에 기록된 당시 상태이며 현재 실차 출력이 아닙니다.</p>
        </section>
      </div>
      <div className="gnss-chart-grid">
        <SignalChart title="GNSS 속도" data={data} series={[{ key: "speed", label: "실제", color: "#16d7e9" }]} minimum={0} maximum={1.2} unit="m/s" />
        <SignalChart title="조향 TARGET / ACTUAL" data={data} series={[{ key: "steeringTarget", label: "목표", color: "#f7d154" }, { key: "steering", label: "실제", color: "#f5f8fa" }]} minimum={-0.55} maximum={0.55} unit="rad" />
        <SignalChart title="경로 CTE" data={data} series={[{ key: "cte", label: "CTE", color: "#ff5d66" }]} minimum={-1} maximum={1} unit="m" />
      </div>
    </main>
  );
}

function VariantsView({ data }: { data: DashboardData }): ReactElement {
  const [selectedVariantIndex, setSelectedVariantIndex] = useState(0);
  const selectedMetadata = data.variantMetadata[selectedVariantIndex];
  return (
    <main className="variants-view">
      <RouteMap data={data} showVariants selectedVariantIndex={selectedVariantIndex} />
      <section className="variant-sidebar">
        <div className="section-title">2 × 2 × 2 경로 슬롯</div>
        <div className="section-subtitle">카드를 눌러 선택 · T자 주차 × 평행 주차 × 종료 차선 변경</div>
        {data.variants.map((path, index) => (
          <button type="button" onClick={() => { setSelectedVariantIndex(index); }} className={`variant-card ${path && path.length > 1 ? "ready" : "waiting-variant"} ${selectedVariantIndex === index ? "selected" : ""}`} key={index}>
            <i style={{ background: VARIANT_COLORS[index] }} />
            <div><strong>{VARIANT_LABELS[index]}</strong><span>{path && path.length > 1 ? `VARIANT ${String(index + 1).padStart(2, "0")} · ${path.length} WP · ${data.variantMetadata[index]?.segmentCount ?? "--"} 구간` : "경로 파일 대기"}</span></div>
            <b>{path && path.length > 1 ? "READY" : "WAITING"}</b>
          </button>
        ))}
        <div className="mission-segment-header">
          <div>
            <div className="section-title">WAYPOINT · MISSION SEGMENTS</div>
            <div className="section-subtitle">{VARIANT_LABELS[selectedVariantIndex]} · CSV index 기준</div>
          </div>
          <b>{selectedMetadata?.segmentCount ?? "--"}</b>
        </div>
        <div className="mission-segment-list">
          {selectedMetadata == undefined ? <div className="segment-waiting">구간 메타데이터 대기</div> : selectedMetadata.segments.map((segment, index) => (
            <article className={`mission-segment ${segment.zone === "ROUTE" ? "route-segment" : "mission-zone-segment"}`} key={`${segment.startIndex}-${segment.zone}`}>
              <i style={{ background: MISSION_ZONE_COLORS[segment.zone] ?? "#a6b8c0" }} />
              <span className="segment-order">{String(index + 1).padStart(2, "0")}</span>
              <div>
                <strong>{segment.zone} <em>{MISSION_ZONE_LABELS[segment.zone] ?? segment.mission}</em></strong>
                <span>WP {segment.startIndex}–{segment.endIndex} · s {segment.startSM.toFixed(1)}–{segment.endSM.toFixed(1)} m · {segment.direction > 0 ? "FWD" : "REV"}</span>
                {segment.eventIds.length > 0 && <small>{segment.eventIds.join(" · ")}</small>}
              </div>
            </article>
          ))}
        </div>
        <p className="variant-warning">현재 8개 파일은 target_speed_mps=0인 형상 검증용입니다. 실차 주행 경로로 사용하지 않습니다.</p>
        <div className="active-route-box"><small>ACTIVE ROUTE</small><strong>{data.activeVariant}</strong></div>
      </section>
    </main>
  );
}

function DashboardPanel({ context }: { context: PanelExtensionContext }): ReactElement {
  const config = asRecord(context.initialState) as PanelConfig | undefined;
  const view: ViewName = config?.view === "perception" || config?.view === "gnss" || config?.view === "variants" ? config.view : "overview";
  const startSec = config?.bagStartSec ?? DEFAULT_BAG_START_SEC;
  const endSec = config?.bagEndSec ?? DEFAULT_BAG_END_SEC;
  const data = useDashboardData(context);
  const live = data.sourceMode === "LIVE DRIVE";
  const title = view === "overview" ? "HL KU FMA MISSION MONITOR" : view === "perception" ? "CAMERA · YOLO MONITOR" : view === "gnss" ? "GNSS · VEHICLE MONITOR" : "8 ROUTE INTEGRATION";

  useEffect(() => {
    context.setDefaultPanelTitle(title);
  }, [context, title]);

  return (
    <div className="dashboard-root">
      <style>{DASHBOARD_CSS}</style>
      <header className="dashboard-header">
        <div><strong>{title}</strong><span>{live ? "CURRENT RUN · LIVE MONITOR" : "RECORDED RUN · SAFE REPLAY"}</span></div>
        <div className="header-status"><span>ROS TIME <b>{live ? new Date(data.nowSec * 1000).toLocaleTimeString("ko-KR", { hour12: false }) : formatDuration(data.nowSec - startSec)}</b></span><span>DATA SOURCE <b>{data.sourceMode}</b></span><span>ROUTE <b>{data.activeVariant}</b></span></div>
      </header>
      <div className="dashboard-body">
        {view === "overview" ? <Overview data={data} /> : view === "perception" ? <PerceptionView data={data} /> : view === "gnss" ? <GnssView data={data} /> : <VariantsView data={data} />}
      </div>
      {!live && <Timeline context={context} data={data} startSec={startSec} endSec={endSec} />}
    </div>
  );
}

const DASHBOARD_CSS = `
  :root { color-scheme: dark; }
  * { box-sizing: border-box; }
  .dashboard-root { --bg:#071116; --panel:#0b161b; --line:#263740; --muted:#76909c; --text:#e7f0f4; --cyan:#16d7e9; --yellow:#f7d154; --red:#ff5d66; width:100%; height:100%; min-width:760px; min-height:520px; overflow:hidden; background:var(--bg); color:var(--text); font-family:Inter,Pretendard,"Noto Sans KR",sans-serif; display:flex; flex-direction:column; }
  .dashboard-header { height:58px; flex:0 0 58px; padding:0 18px; border-bottom:1px solid var(--line); display:flex; align-items:center; justify-content:space-between; background:#071015; }
  .dashboard-header>div:first-child { display:flex; align-items:baseline; gap:14px; }
  .dashboard-header strong { font-size:17px; letter-spacing:.04em; }
  .dashboard-header span { color:var(--muted); font-size:10px; letter-spacing:.05em; }
  .header-status { display:flex; gap:22px; text-align:right; }
  .header-status span { display:flex; flex-direction:column; gap:3px; }
  .header-status b { color:var(--cyan); font-size:12px; max-width:440px; white-space:nowrap; overflow:hidden; text-overflow:ellipsis; }
  .dashboard-body { min-height:0; flex:1; padding:6px; }
  .dashboard-grid { height:100%; display:grid; grid-template-columns:46% 54%; gap:6px; }
  .left-stack,.right-stack { min-height:0; display:flex; flex-direction:column; gap:6px; }
  .map-grid,.camera-grid { display:grid; grid-template-columns:1fr 1fr; gap:6px; min-height:0; }
  .map-grid { flex:0 0 42%; }
  .overview-variant-map { flex:0 0 42%; min-height:0; }
  .overview-variant-map>.map-panel { width:100%; height:100%; }
  .camera-grid { flex:0 0 31%; }
  .panel { position:relative; min-width:0; min-height:0; overflow:hidden; border:1px solid var(--line); background:var(--panel); }
  .section-title { color:#b7d4df; font-size:11px; font-weight:750; letter-spacing:.045em; }
  .section-subtitle { color:var(--muted); font-size:9px; margin-top:4px; white-space:nowrap; overflow:hidden; text-overflow:ellipsis; }
  .map-live-metrics { color:#a7bbc3; }
  .map-heading { position:absolute; z-index:2; top:0; left:0; right:0; padding:10px 12px; display:flex; justify-content:space-between; pointer-events:none; }
  .source-badge { align-self:flex-start; font-size:8px; color:var(--yellow); border:1px solid #675d31; border-radius:3px; padding:3px 5px; }
  .compact-variant-selector { position:absolute; z-index:3; top:43px; left:12px; display:flex; gap:4px; padding:4px; border:1px solid var(--line); background:#071015e8; }
  .compact-variant-selector button { width:28px; height:22px; border:1px solid #344750; border-radius:2px; color:#89a0a9; background:#0d1a20; font-size:8px; cursor:pointer; }
  .compact-variant-selector button:hover,.compact-variant-selector button.selected { border-color:var(--cyan); color:#071116; background:var(--cyan); }
  .compact-variant-selector button:disabled { opacity:.35; cursor:wait; }
  .route-svg { width:100%; height:100%; display:block; }
  .source-course-inset { position:absolute; z-index:3; right:8px; bottom:8px; width:34%; min-width:190px; height:52%; padding:7px; border:1px solid #3a5661; background:#071015ed; display:grid; grid-template-rows:auto 1fr 4px auto auto; gap:4px; box-shadow:0 0 16px #0008; }
  .source-course-heading { display:flex; justify-content:space-between; gap:6px; font-size:8px; color:#a9bbc2; }
  .source-course-heading b { color:var(--yellow); white-space:nowrap; }
  .source-course-inset svg { width:100%; height:100%; min-height:0; }
  .source-progress-track { height:4px; background:#263942; overflow:hidden; }
  .source-progress-track i { display:block; height:100%; background:var(--yellow); }
  .source-course-inset>strong { color:#e7f0f4; font-size:9px; font-variant-numeric:tabular-nums; }
  .source-course-inset>span { color:#8299a3; font-size:7px; font-variant-numeric:tabular-nums; }
  .map-legend { position:absolute; left:10px; bottom:8px; display:flex; gap:7px; padding:5px 7px; border:1px solid var(--line); background:#071015dd; font-size:8px; }
  .map-legend span { display:flex; align-items:center; gap:4px; color:#a8bac2; }
  .map-legend i { width:17px; height:3px; display:inline-block; }
  .map-legend i.start-dot,.map-legend i.end-dot { width:7px; height:7px; border-radius:50%; }
  .map-legend i.start-dot { background:#31d59b; }
  .map-legend i.end-dot { background:#ff5d66; }
  .mission-map-legend { right:10px; flex-wrap:wrap; justify-content:center; gap:5px 10px; }
  .mission-map-legend i { width:12px; }
  .mission-map-legend i.hill-stop-dot { width:8px; height:8px; border-radius:50%; background:#ff5d66; border:1px solid #f7d154; }
  .camera-panel { display:flex; flex-direction:column; }
  .camera-panel .section-title { padding:8px 10px; flex:0 0 30px; }
  .camera-canvas { flex:1; min-height:0; width:100%; height:calc(100% - 30px); object-fit:contain; background:#03080a; }
  .waiting { flex:1; display:flex; align-items:center; justify-content:center; color:#5f7680; font-size:11px; background:linear-gradient(135deg,#0a1419,#111c21); }
  .fsm-panel { flex:1; padding:10px; display:flex; flex-direction:column; }
  .fsm-header { display:flex; justify-content:space-between; align-items:center; flex:0 0 44px; }
  .selected-state { min-width:145px; display:grid; grid-template-columns:auto auto; column-gap:8px; align-items:center; text-align:right; }
  .selected-state small { grid-column:1/3; color:var(--muted); font-size:8px; }
  .selected-state strong { color:var(--cyan); font-size:14px; }
  .selected-state span { color:var(--yellow); font-size:8px; }
  .fsm-grid { flex:1; min-height:0; display:grid; grid-template-columns:repeat(3,1fr); gap:5px; }
  .fsm-card { position:relative; min-height:0; padding:8px 10px; border:1px solid #2b3c44; border-left:5px solid #5d6c73; border-radius:3px; display:flex; flex-direction:column; justify-content:center; }
  .fsm-card.active { border-color:#148895; border-left-color:var(--cyan); background:#0c2228; box-shadow:inset 0 0 18px #0b748322; }
  .fsm-card strong { color:#b6c6cd; font-size:11px; }
  .fsm-card b { color:#71858d; font-size:13px; margin:3px 0; }
  .fsm-card.active b { color:var(--cyan); }
  .fsm-card span,.fsm-card em { color:#687b83; font-size:8px; font-style:normal; }
  .fsm-card em { position:absolute; right:8px; bottom:7px; }
  .fsm-card.active em { color:var(--yellow); }
  .fsm-index { position:absolute; right:8px; top:6px; color:#53666f!important; }
  .summary-panel { flex:0 0 15%; display:grid; grid-template-columns:40% 60%; }
  .mission-summary { padding:10px 13px; border-right:1px solid var(--line); display:flex; flex-direction:column; justify-content:center; }
  .mission-summary small { color:var(--muted); font-size:8px; }
  .mission-summary strong { color:var(--cyan); font-size:18px; margin:3px 0; }
  .mission-summary span { color:#a9bac1; font-size:9px; }
  .summary-metrics { display:grid; grid-template-columns:repeat(3,1fr); }
  .metric-card { padding:8px 10px; border-right:1px solid #1c2d35; border-bottom:1px solid #1c2d35; display:flex; flex-direction:column; justify-content:center; }
  .metric-card span { color:var(--muted); font-size:8px; }
  .metric-card strong { color:#e5edf0; font-size:13px; margin-top:3px; }
  .metric-card.good strong { color:#31d59b; } .metric-card.warn strong { color:var(--yellow); } .metric-card.bad strong { color:var(--red); }
  .chart-panel { flex:1; padding:7px 10px 2px; display:flex; flex-direction:column; }
  .chart-heading { display:flex; justify-content:space-between; align-items:center; flex:0 0 24px; }
  .chart-values { display:flex; gap:10px; font-size:9px; }
  .chart-svg { width:100%; flex:1; min-height:0; }
  .timeline { height:58px; flex:0 0 58px; border-top:1px solid #31444d; background:#081217; display:flex; align-items:center; gap:10px; padding:8px 14px; }
  .timeline input[type=range] { flex:1; accent-color:var(--cyan); cursor:pointer; }
  .timeline button { color:#b9cbd2; background:#111f25; border:1px solid #344750; border-radius:3px; min-width:38px; height:28px; font-size:10px; cursor:pointer; }
  .timeline button:hover,.timeline button.selected { color:#071116; border-color:var(--cyan); background:var(--cyan); }
  .timeline button:disabled { opacity:.45; cursor:wait; }
  .timeline-controls,.rate-controls { display:flex; gap:4px; }
  .play-button { font-size:13px!important; }
  .timeline-time { color:#a8bbc3; font-variant-numeric:tabular-nums; font-size:10px; min-width:80px; text-align:center; }
  .detail-view,.gnss-view,.variants-view { height:100%; display:flex; gap:6px; min-height:0; }
  .detail-view { flex-direction:column; }
  .detail-camera-grid { flex:0 0 58%; display:grid; grid-template-columns:1fr 1fr; gap:6px; min-height:0; }
  .perception-bottom { flex:1; min-height:0; display:grid; grid-template-columns:32% 48% 20%; gap:6px; }
  .perception-bottom .fsm-grid { grid-template-columns:repeat(3,1fr); }
  .readiness-panel { padding:12px; display:flex; flex-direction:column; gap:6px; }
  .readiness-panel .metric-card { min-height:58px; border:1px solid var(--line); }
  .readiness-panel p,.safety-panel p { color:var(--muted); font-size:9px; line-height:1.5; }
  .gnss-view { flex-direction:column; }
  .status-card-grid { flex:0 0 88px; display:grid; grid-template-columns:repeat(8,1fr); gap:6px; }
  .status-card-grid .metric-card { border:1px solid var(--line); background:var(--panel); }
  .status-card-grid .metric-card strong { font-size:15px; }
  .gnss-main-grid { flex:1; min-height:0; display:grid; grid-template-columns:70% 30%; gap:6px; }
  .safety-panel { padding:13px; }
  .safety-panel>strong { display:block; color:var(--cyan); margin:12px 0; font-size:16px; }
  .safety-row { display:flex; justify-content:space-between; padding:9px 0; border-top:1px solid var(--line); color:var(--muted); font-size:10px; }
  .safety-row b { color:var(--text); } .good-text{color:#31d59b!important}.bad-text{color:var(--red)!important}
  .gnss-chart-grid { flex:0 0 31%; display:grid; grid-template-columns:repeat(3,1fr); gap:6px; min-height:0; }
  .variants-view { display:grid; grid-template-columns:1fr 390px; }
  .variant-sidebar { min-height:0; overflow:auto; padding:12px; border:1px solid var(--line); background:var(--panel); }
  .variant-card { width:100%; margin-top:7px; padding:9px; display:grid; grid-template-columns:6px 1fr auto; gap:10px; align-items:center; border:1px solid #2b3c44; border-radius:3px; color:inherit; background:#0b161b; font:inherit; text-align:left; cursor:pointer; }
  .variant-card:hover { border-color:#477080; background:#102027; }
  .variant-card i { width:5px; height:31px; }
  .variant-card div { display:flex; flex-direction:column; gap:3px; }
  .variant-card strong { font-size:10px; }.variant-card span { color:var(--muted); font-size:8px; }.variant-card b { color:#6c7d84; font-size:8px; }
  .variant-card.ready { border-color:#246859; }.variant-card.ready b { color:#31d59b; }
  .variant-card.selected { border-color:var(--cyan); background:#0c2228; box-shadow:inset 0 0 14px #0b748322; }
  .variant-card.selected strong { color:var(--cyan); }
  .mission-segment-header { margin-top:14px; padding:10px 0 7px; border-top:1px solid var(--line); display:flex; align-items:center; justify-content:space-between; }
  .mission-segment-header>b { min-width:32px; padding:5px; border:1px solid #37606d; color:var(--cyan); text-align:center; font-size:11px; }
  .mission-segment-list { max-height:420px; overflow:auto; border:1px solid var(--line); background:#071015; }
  .mission-segment { position:relative; min-height:44px; padding:7px 8px 7px 35px; border-bottom:1px solid #1b2b33; display:flex; gap:8px; }
  .mission-segment:last-child { border-bottom:0; }
  .mission-segment>i { position:absolute; left:0; top:0; bottom:0; width:5px; }
  .mission-segment .segment-order { position:absolute; left:10px; top:9px; color:#607780; font-size:8px; }
  .mission-segment>div { min-width:0; display:flex; flex-direction:column; gap:3px; }
  .mission-segment strong { color:#c7d7dd; font-size:9px; }
  .mission-segment strong em { margin-left:5px; color:#718a95; font-size:8px; font-style:normal; font-weight:500; }
  .mission-segment span { color:#8ba0a9; font-size:8px; font-variant-numeric:tabular-nums; }
  .mission-segment small { color:var(--yellow); font-size:7px; line-height:1.35; overflow-wrap:anywhere; }
  .mission-zone-segment { background:#0c1c22; }
  .route-segment { opacity:.72; }
  .segment-waiting { padding:18px; color:var(--muted); text-align:center; font-size:9px; }
  .variant-warning { margin:10px 0 0; padding:9px; border:1px solid #675d31; color:var(--yellow); background:#241f0d; font-size:9px; line-height:1.45; }
  .active-route-box { margin-top:12px; padding:12px; border:1px solid #148895; background:#0c2228; display:flex; flex-direction:column; gap:4px; }
  .active-route-box small { color:var(--muted); }.active-route-box strong { color:var(--cyan); font-size:12px; }
  @media(max-width:1100px){.dashboard-grid{grid-template-columns:1fr}.right-stack{display:none}.map-grid,.overview-variant-map{flex:0 0 45%}.status-card-grid{grid-template-columns:repeat(4,1fr);flex-basis:150px}.variants-view{grid-template-columns:1fr 300px}.header-status span:last-child{display:none}}
`;

export function initDashboardPanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<DashboardPanel context={context} />);
  return () => {
    root.unmount();
  };
}
