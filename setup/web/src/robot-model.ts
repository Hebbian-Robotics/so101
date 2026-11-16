import {
  Component,
  createElement,
  type ReactNode,
  useEffect,
  useState,
} from "react";
import * as THREE from "three";
import URDFLoader, { type URDFRobot } from "urdf-loader";

type LoadedRobot =
  | { robot: URDFRobot; error: null }
  | { robot: null; error: string };

function disposeRobot(robot: URDFRobot): void {
  const geometries = new Set<THREE.BufferGeometry>();
  const materials = new Set<THREE.Material>();
  robot.traverse((object) => {
    if (!(object instanceof THREE.Mesh)) return;
    geometries.add(object.geometry);
    for (const material of Array.isArray(object.material)
      ? object.material
      : [object.material])
      materials.add(material);
  });
  for (const geometry of geometries) geometry.dispose();
  for (const material of materials) material.dispose();
}

export function useRobotModel(
  modelUrl = "robot/so101/robot.urdf",
): LoadedRobot | null {
  const [loaded, setLoaded] = useState<{
    modelUrl: string;
    result: LoadedRobot;
  } | null>(null);
  useEffect(() => {
    const manager = new THREE.LoadingManager();
    let robot: URDFRobot | null = null;
    let cancelled = false;
    let failed = false;
    let completed = false;
    manager.onError = () => {
      failed = true;
    };
    manager.onLoad = () => {
      completed = true;
      if (cancelled) {
        if (robot) disposeRobot(robot);
      } else if (failed || !robot) {
        if (robot) disposeRobot(robot);
        setLoaded({
          modelUrl,
          result: {
            robot: null,
            error:
              "Could not load the SO-101 model. Joint values remain available.",
          },
        });
      } else {
        for (const joint of Object.values(robot.joints))
          joint.ignoreLimits = true;
        setLoaded({ modelUrl, result: { robot, error: null } });
      }
    };
    new URDFLoader(manager).load(
      modelUrl,
      (loadedRobot) => {
        robot = loadedRobot;
      },
      undefined,
      () => {
        failed = true;
      },
    );
    return () => {
      cancelled = true;
      if (completed && robot && !failed) disposeRobot(robot);
    };
  }, [modelUrl]);
  return loaded?.modelUrl === modelUrl ? loaded.result : null;
}

function gripperAngle(opening: number): number {
  return -0.174533 + THREE.MathUtils.clamp(opening, 0, 1) * (1.5 + 0.174533);
}

export function colorRobot(
  robot: URDFRobot,
  color: number,
  opacity: number,
): void {
  robot.traverse((object) => {
    if (!(object instanceof THREE.Mesh)) return;
    object.material = new THREE.MeshStandardMaterial({
      color,
      roughness: 0.8,
      metalness: 0.05,
      side: THREE.DoubleSide,
      transparent: opacity < 1,
      opacity,
      depthWrite: opacity === 1,
    });
  });
}

export function disposeRobotMaterials(robot: URDFRobot): void {
  robot.traverse((object) => {
    if (!(object instanceof THREE.Mesh)) return;
    for (const material of Array.isArray(object.material)
      ? object.material
      : [object.material])
      material.dispose();
  });
}

export function setRobotJoints(
  robot: URDFRobot,
  jointNames: string[],
  joints: readonly (number | null)[],
  opening: number | null,
  measuredGripperAngle?: number,
): void {
  jointNames.forEach((name, index) => {
    const value = joints[index];
    if (value !== null && value !== undefined && Number.isFinite(value))
      robot.setJointValue(name, value);
  });
  if (measuredGripperAngle !== undefined)
    robot.setJointValue("gripper", measuredGripperAngle);
  else if (opening !== null)
    robot.setJointValue("gripper", gripperAngle(opening));
}

export class SceneErrorBoundary extends Component<
  { children: ReactNode },
  { failed: boolean }
> {
  override state = { failed: false };
  static getDerivedStateFromError() {
    return { failed: true };
  }
  override render() {
    return this.state.failed
      ? createElement(
          "p",
          null,
          "3D view needs WebGL. Joint values remain available.",
        )
      : this.props.children;
  }
}
