import { angleDistance, assertFiniteNumber, toLocal, wrapAngle } from './math.js';

export const DEFAULT_HORIZONS = [0.2, 0.5, 1.0];
export const TRAJECTORY_TOLERANCE = 0.08;

const POSE_DIMENSIONS = [
  'leftFoot.x',
  'leftFoot.y',
  'rightFoot.x',
  'rightFoot.y',
];
const MOTION_DIMENSIONS = [
  'velocity.x',
  'velocity.y',
  'acceleration.x',
  'acceleration.y',
];

export function validateFrame(frame, label = 'frame') {
  if (frame === null || typeof frame !== 'object') {
    throw new TypeError(`${label} must be an object`);
  }
  assertFiniteNumber(frame.time, `${label}.time`);
  if (frame.root === null || typeof frame.root !== 'object') {
    throw new TypeError(`${label}.root is required`);
  }
  assertFiniteNumber(frame.root.x, `${label}.root.x`);
  assertFiniteNumber(frame.root.y, `${label}.root.y`);
  assertFiniteNumber(frame.root.heading, `${label}.root.heading`);
  if (frame.velocity === null || typeof frame.velocity !== 'object') {
    throw new TypeError(`${label}.velocity is required (a zero vector is allowed)`);
  }
  assertFiniteNumber(frame.velocity.x, `${label}.velocity.x`);
  assertFiniteNumber(frame.velocity.y, `${label}.velocity.y`);
  validateOptionalFoot(frame.feet, label);
  if (frame.trajectory !== undefined) {
    if (!Array.isArray(frame.trajectory)) {
      throw new TypeError(`${label}.trajectory must be an array`);
    }
    frame.trajectory.forEach((point, index) => {
      const pointLabel = `${label}.trajectory[${index}]`;
      if (point === null || typeof point !== 'object') {
        throw new TypeError(`${pointLabel} must be an object`);
      }
      assertFiniteNumber(point.t, `${pointLabel}.t`);
      assertFiniteNumber(point.x, `${pointLabel}.x`);
      assertFiniteNumber(point.y, `${pointLabel}.y`);
      assertFiniteNumber(point.heading, `${pointLabel}.heading`);
    });
  }
  validateOptionalVector(frame.acceleration, `${label}.acceleration`);
  validateOptionalVector(frame.prevVelocity, `${label}.prevVelocity`);
  validateOptionalVector(frame.nextVelocity, `${label}.nextVelocity`);
}

function validateOptionalFoot(feet, label) {
  if (feet === undefined || feet === null) return;
  if (typeof feet !== 'object') {
    throw new TypeError(`${label}.feet must be an object when present`);
  }
  for (const side of ['left', 'right']) {
    const foot = feet[side];
    if (foot === null || foot === undefined) continue;
    if (typeof foot !== 'object') {
      throw new TypeError(`${label}.feet.${side} must be an object or null`);
    }
    assertFiniteNumber(foot.x, `${label}.feet.${side}.x`);
    assertFiniteNumber(foot.y, `${label}.feet.${side}.y`);
  }
}

function validateOptionalVector(vector, label) {
  if (vector === undefined || vector === null) return;
  if (typeof vector !== 'object') {
    throw new TypeError(`${label} must be an object when present`);
  }
  assertFiniteNumber(vector.x, `${label}.x`);
  assertFiniteNumber(vector.y, `${label}.y`);
}

function resolveAcceleration(frame, prevFrame, nextFrame, frameDuration) {
  if (frame.acceleration) {
    return { vector: frame.acceleration, present: true };
  }
  if (nextFrame) {
    const dt = nextFrame.time - frame.time;
    if (dt > 0) {
      return {
        vector: {
          x: (nextFrame.velocity.x - frame.velocity.x) / dt,
          y: (nextFrame.velocity.y - frame.velocity.y) / dt,
        },
        present: true,
      };
    }
  }
  if (prevFrame) {
    const dt = frame.time - prevFrame.time;
    if (dt > 0) {
      return {
        vector: {
          x: (frame.velocity.x - prevFrame.velocity.x) / dt,
          y: (frame.velocity.y - prevFrame.velocity.y) / dt,
        },
        present: true,
      };
    }
  }
  if (frameDuration > 0 && frame.nextVelocity) {
    return {
      vector: {
        x: (frame.nextVelocity.x - frame.velocity.x) / frameDuration,
        y: (frame.nextVelocity.y - frame.velocity.y) / frameDuration,
      },
      present: true,
    };
  }
  if (frameDuration > 0 && frame.prevVelocity) {
    return {
      vector: {
        x: (frame.velocity.x - frame.prevVelocity.x) / frameDuration,
        y: (frame.velocity.y - frame.prevVelocity.y) / frameDuration,
      },
      present: true,
    };
  }
  return { vector: { x: 0, y: 0 }, present: false };
}

function sampleTrajectory(frame, horizons, tolerance) {
  const values = [];
  const mask = [];
  const points = frame.trajectory ?? [];
  for (const horizon of horizons) {
    const targetTime = frame.time + horizon;
    let chosen = null;
    let bestDelta = Infinity;
    for (const point of points) {
      const delta = Math.abs(point.t - targetTime);
      if (delta <= tolerance && delta < bestDelta) {
        chosen = point;
        bestDelta = delta;
      }
    }
    if (chosen === null) {
      values.push(0, 0, 0);
      mask.push(false, false, false);
      continue;
    }
    const local = toLocal(
      chosen.x - frame.root.x,
      chosen.y - frame.root.y,
      frame.root.heading,
    );
    values.push(local.x, local.y, wrapAngle(angleDistance(chosen.heading, frame.root.heading)));
    mask.push(true, true, true);
  }
  return { values, mask };
}

export function extractFrameFeatures(frame, options = {}) {
  const {
    prevFrame = null,
    nextFrame = null,
    horizons = DEFAULT_HORIZONS,
    tolerance = TRAJECTORY_TOLERANCE,
    frameDuration = 0,
  } = options;
  validateFrame(frame);

  const poseValues = [];
  const poseMask = [];
  const feet = frame.feet ?? {};
  for (const side of ['left', 'right']) {
    const foot = feet[side];
    if (foot === null || foot === undefined) {
      poseValues.push(0, 0);
      poseMask.push(false, false);
      continue;
    }
    const local = toLocal(
      foot.x - frame.root.x,
      foot.y - frame.root.y,
      frame.root.heading,
    );
    poseValues.push(local.x, local.y);
    poseMask.push(true, true);
  }

  const localVelocity = toLocal(frame.velocity.x, frame.velocity.y, frame.root.heading);
  const acceleration = resolveAcceleration(frame, prevFrame, nextFrame, frameDuration);
  const localAcceleration = acceleration.present
    ? toLocal(acceleration.vector.x, acceleration.vector.y, frame.root.heading)
    : { x: 0, y: 0 };
  const motionValues = [
    localVelocity.x,
    localVelocity.y,
    localAcceleration.x,
    localAcceleration.y,
  ];
  const motionMask = [true, true, acceleration.present, acceleration.present];

  const trajectory = sampleTrajectory(frame, horizons, tolerance);

  return {
    pose: { values: poseValues, mask: poseMask, dimensions: [...POSE_DIMENSIONS] },
    motion: { values: motionValues, mask: motionMask, dimensions: [...MOTION_DIMENSIONS] },
    trajectory: {
      values: trajectory.values,
      mask: trajectory.mask,
      dimensions: horizons.flatMap((horizon) => [
        `trajectory@${horizon}s.x`,
        `trajectory@${horizon}s.y`,
        `trajectory@${horizon}s.heading`,
      ]),
    },
  };
}
