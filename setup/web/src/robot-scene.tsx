import { Canvas, useThree } from "@react-three/fiber";
import { useEffect, useMemo, useState } from "react";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import type { URDFRobot } from "urdf-loader";
import {
  colorRobot,
  disposeRobotMaterials,
  SceneErrorBoundary,
  setRobotJoints,
  useRobotModel,
} from "./robot-model.js";

type Position = [number, number, number];
type SceneProps = {
  modelUrl?: string;
  jointNames: string[];
  measuredJoints: (number | null)[];
  gripperAngleRadians?: number;
  referenceJoints: number[] | null;
  referenceOpening: number;
  cameraPosition: Position;
  cameraTarget: Position;
};

function Controls({
  cameraPosition,
  cameraTarget,
  resetCount,
}: Pick<SceneProps, "cameraPosition" | "cameraTarget"> & {
  resetCount: number;
}) {
  const { camera, gl, invalidate } = useThree();
  // biome-ignore lint/correctness/useExhaustiveDependencies: resetCount repeats the camera reset requested by the operator.
  useEffect(() => {
    camera.up.set(0, 0, 1);
    const controls = new OrbitControls(camera, gl.domElement);
    camera.position.set(...cameraPosition);
    controls.target.set(...cameraTarget);
    controls.update();
    const onCameraChange = () => invalidate();
    controls.addEventListener("change", onCameraChange);
    return () => {
      controls.removeEventListener("change", onCameraChange);
      controls.dispose();
    };
  }, [camera, gl, invalidate, cameraPosition, cameraTarget, resetCount]);
  return null;
}

function Models({ template, ...props }: SceneProps & { template: URDFRobot }) {
  const { invalidate } = useThree();
  const models = useMemo(() => {
    const measured = template.clone() as URDFRobot;
    const reference = template.clone() as URDFRobot;
    colorRobot(measured, 0x999999, 1);
    colorRobot(reference, 0x087f8c, 0.4);
    return { measured, reference };
  }, [template]);
  useEffect(
    () => () => {
      disposeRobotMaterials(models.measured);
      disposeRobotMaterials(models.reference);
    },
    [models],
  );
  useEffect(() => {
    models.measured.visible =
      props.measuredJoints.length === 5 &&
      props.measuredJoints.every((position) => position !== null);
    setRobotJoints(
      models.measured,
      props.jointNames,
      props.measuredJoints,
      null,
      props.gripperAngleRadians,
    );
    models.reference.visible = props.referenceJoints !== null;
    if (props.referenceJoints)
      setRobotJoints(
        models.reference,
        props.jointNames,
        props.referenceJoints,
        props.referenceOpening,
      );
    invalidate();
  }, [
    models,
    props.measuredJoints,
    props.jointNames,
    props.gripperAngleRadians,
    props.referenceJoints,
    props.referenceOpening,
    invalidate,
  ]);
  return (
    <>
      <primitive object={models.measured} />
      <primitive object={models.reference} />
    </>
  );
}

export function RobotScene(props: SceneProps) {
  const loaded = useRobotModel(props.modelUrl);
  const [resetCount, setResetCount] = useState(0);
  return (
    <section className="robot-scene" aria-label="Robot visualization">
      <header>
        <h2>Robot · 3D</h2>
        <button
          type="button"
          onClick={() => setResetCount((count) => count + 1)}
        >
          Reset view
        </button>
      </header>
      <div className="robot-scene-canvas">
        {!loaded ? (
          <p>Loading SO-101 model…</p>
        ) : !loaded.robot ? (
          <p>{loaded.error}</p>
        ) : (
          <SceneErrorBoundary>
            <Canvas
              frameloop="demand"
              camera={{ fov: 40, near: 0.01, far: 30 }}
              aria-label="Robot model: drag to orbit, scroll to zoom"
            >
              <color attach="background" args={["#eeeeee"]} />
              <hemisphereLight args={[0xffffff, 0x888888, 1.5]} />
              <directionalLight position={[2, -3, 4]} intensity={1.5} />
              <gridHelper
                args={[2, 20, 0xaaaaaa, 0xd8d8d8]}
                rotation={[Math.PI / 2, 0, 0]}
              />
              <axesHelper args={[0.2]} />
              <Models template={loaded.robot} {...props} />
              <Controls
                cameraPosition={props.cameraPosition}
                cameraTarget={props.cameraTarget}
                resetCount={resetCount}
              />
            </Canvas>
          </SceneErrorBoundary>
        )}
      </div>
    </section>
  );
}
