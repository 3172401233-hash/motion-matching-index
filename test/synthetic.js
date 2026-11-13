import { DEFAULT_HORIZONS } from '../src/features.js';

export const FRAME_INTERVAL = 0.1;

function footPositions(progress, side) {
  const sign = side === 'left' ? 1 : -1;
  const stride = Math.sin(progress * 2 * Math.PI) * 0.25;
  return {
    x: stride + sign * 0.03,
    y: sign * 0.12,
  };
}

function buildTrajectory(frame, stateAt, horizonTimes, frameCount, loop = false) {
  return horizonTimes.map((horizon) => {
    const absoluteTime = frame.time + horizon;
    let state;
    if (loop) {
      const wrappedIndex = (frame.time + horizon) / FRAME_INTERVAL;
      state = stateAt(((wrappedIndex % frameCount) + frameCount) % frameCount);
      return { t: absoluteTime, ...state };
    }
    const futureIndex = Math.round(absoluteTime / FRAME_INTERVAL);
    if (futureIndex >= frameCount) return null;
    state = stateAt(futureIndex);
    return { t: absoluteTime, ...state };
  }).filter((point) => point !== null);
}

function makeClip({
  id,
  frameCount,
  rootAt,
  velocityAt,
  progressAt,
  loop = false,
  omitFeet = false,
  omitRightFoot = false,
  omitTrajectory = false,
  horizons = DEFAULT_HORIZONS,
}) {
  const states = Array.from({ length: frameCount }, (_, index) => {
    const root = rootAt(index);
    const velocity = velocityAt(index);
    const progress = progressAt(index);
    const frame = {
      time: index * FRAME_INTERVAL,
      root,
      velocity,
      feet: omitFeet ? undefined : {
        left: {
          x: root.x + footPositions(progress, 'left').x,
          y: root.y + footPositions(progress, 'left').y,
        },
        right: omitRightFoot ? null : {
          x: root.x + footPositions(progress + 0.5, 'right').x,
          y: root.y + footPositions(progress + 0.5, 'right').y,
        },
      },
    };
    return frame;
  });

  const stateAt = (index) => ({
    x: rootAt(index).x,
    y: rootAt(index).y,
    heading: rootAt(index).heading,
  });

  const frames = states.map((frame, index) => ({
    ...frame,
    trajectory: omitTrajectory ? undefined : buildTrajectory(
      frame,
      stateAt,
      horizons,
      frameCount,
      loop,
    ),
  }));

  return { id, loop, frames };
}

export function straightClip(id = 'straight', overrides = {}) {
  const frameCount = 24;
  const speed = 1.4;
  return makeClip({
    id,
    frameCount,
    rootAt: (index) => ({ x: index * speed * FRAME_INTERVAL, y: 0, heading: 0 }),
    velocityAt: () => ({ x: speed, y: 0 }),
    progressAt: (index) => index / 10,
    ...overrides,
  });
}

export function turnClip(id = 'turn') {
  const frameCount = 24;
  const turnRate = 0.9;
  const speed = 1.0;
  const headingAt = (index) => index * turnRate * FRAME_INTERVAL;
  return makeClip({
    id,
    frameCount,
    rootAt: (index) => {
      const heading = headingAt(index);
      return {
        x: Math.sin(heading) * speed / turnRate,
        y: (1 - Math.cos(heading)) * speed / turnRate,
        heading,
      };
    },
    velocityAt: (index) => {
      const heading = headingAt(index);
      return { x: speed * Math.cos(heading), y: speed * Math.sin(heading) };
    },
    progressAt: (index) => index / 12,
  });
}

export function stopClip(id = 'stop') {
  const frameCount = 20;
  const initialSpeed = 1.2;
  const deceleration = 1.0;
  const speedAt = (index) => Math.max(0, initialSpeed - deceleration * index * FRAME_INTERVAL);
  const positionAt = (index) => {
    let position = 0;
    for (let step = 0; step < index; step += 1) {
      position += speedAt(step) * FRAME_INTERVAL;
    }
    return position;
  };
  return makeClip({
    id,
    frameCount,
    rootAt: (index) => ({ x: positionAt(index), y: 0, heading: 0 }),
    velocityAt: (index) => ({ x: speedAt(index), y: 0 }),
    progressAt: (index) => index / 10,
  });
}

export function loopWalkClip(id = 'loop-walk') {
  const frameCount = 20;
  const radius = 1.0;
  const angleAt = (index) => (2 * Math.PI * index) / frameCount;
  return makeClip({
    id,
    frameCount,
    loop: true,
    rootAt: (index) => {
      const angle = angleAt(index);
      return {
        x: radius * Math.cos(angle),
        y: radius * Math.sin(angle),
        heading: angle + Math.PI / 2,
      };
    },
    velocityAt: (index) => {
      const angle = angleAt(index);
      const speed = (2 * Math.PI * radius) / (frameCount * FRAME_INTERVAL);
      return { x: -speed * Math.sin(angle), y: speed * Math.cos(angle) };
    },
    progressAt: (index) => index / 10,
  });
}

export function buildFixture() {
  return [straightClip(), turnClip(), stopClip(), loopWalkClip()];
}
