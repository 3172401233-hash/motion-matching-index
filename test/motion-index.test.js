import assert from 'node:assert/strict';
import { test } from 'node:test';

import {
  buildMotionIndex,
  circularFrameDistance,
  queryMotionIndex,
} from '../src/index.js';
import {
  FRAME_INTERVAL,
  buildFixture,
  loopWalkClip,
  stopClip,
  straightClip,
  turnClip,
} from './synthetic.js';

const EPSILON = 1e-9;

test('a straight-walk query prefers straight-walk frames over turn and stop frames', () => {
  const index = buildMotionIndex(buildFixture());
  const straight = straightClip();
  const candidates = queryMotionIndex(index, straight.frames[8], { topK: index.size });

  assert.ok(candidates.length > 0);
  assert.equal(candidates[0].clipId, 'straight');
  assert.ok(candidates[0].cost < EPSILON);
  const firstOther = candidates.find((candidate) => candidate.clipId !== 'straight');
  assert.ok(firstOther);
  assert.ok(candidates[0].cost < firstOther.cost);
  const ranksByClip = new Map();
  candidates.forEach((candidate, rank) => {
    if (!ranksByClip.has(candidate.clipId)) ranksByClip.set(candidate.clipId, rank);
  });
  assert.equal(ranksByClip.get('straight'), 0);
});

test('a turn query prefers the turn clip', () => {
  const index = buildMotionIndex(buildFixture());
  const turn = turnClip();
  const candidates = queryMotionIndex(index, turn.frames[10], { topK: 3 });
  assert.equal(candidates[0].clipId, 'turn');
  assert.ok(candidates[0].cost < EPSILON);
});

test('excludedClipIds removes the just-played clip from all returned candidates', () => {
  const index = buildMotionIndex(buildFixture());
  const straight = straightClip();
  const candidates = queryMotionIndex(index, straight.frames[8], {
    topK: index.size,
    excludedClipIds: ['straight'],
  });

  assert.ok(candidates.length > 0);
  assert.ok(candidates.every((candidate) => candidate.clipId !== 'straight'));
  assert.deepEqual(
    [...new Set(candidates.map((candidate) => candidate.clipId))].sort(),
    index.clipIds.filter((clipId) => clipId !== 'straight').sort(),
  );
});

test('excluding every eligible clip returns an empty list instead of failing', () => {
  const index = buildMotionIndex(buildFixture());
  const candidates = queryMotionIndex(index, straightClip().frames[4], {
    excludedClipIds: ['straight', 'turn', 'stop', 'loop-walk'],
  });
  assert.deepEqual(candidates, []);
});

test('weights change the ranking order', () => {
  const clips = buildFixture();
  const trajectoryIndex = buildMotionIndex(clips, {
    weights: { pose: 0, motion: 0, trajectory: 1 },
  });
  const poseIndex = buildMotionIndex(clips, {
    weights: { pose: 1, motion: 0, trajectory: 0 },
  });
  const turn = turnClip();

  const trajectoryRanking = queryMotionIndex(trajectoryIndex, turn.frames[6], { topK: trajectoryIndex.size });
  const poseRanking = queryMotionIndex(poseIndex, turn.frames[6], { topK: 12 });

  const bestByTrajectory = trajectoryRanking[0];
  const poseOrder = poseRanking.map((candidate) => candidate.clipId);
  const trajectoryOrder = trajectoryRanking.map((candidate) => candidate.clipId);
  assert.ok(Number.isFinite(bestByTrajectory.cost));
  assert.notDeepEqual(trajectoryOrder, poseOrder);
  assert.notEqual(
    trajectoryRanking.findIndex((candidate) => candidate.clipId === 'turn'),
    poseRanking.findIndex((candidate) => candidate.clipId === 'turn'),
  );
});

test('a zero weight group does not change cost even when its scales change', () => {
  const clips = buildFixture();
  const buildWith = (trajectoryWeight) => buildMotionIndex(clips, {
    weights: { pose: 1, motion: 1, trajectory: trajectoryWeight },
    scales: {
      'trajectory@0.2s.x': 100,
      'trajectory@0.2s.y': 100,
      'trajectory@0.2s.heading': 100,
      'trajectory@0.5s.x': 100,
      'trajectory@0.5s.y': 100,
      'trajectory@0.5s.heading': 100,
      'trajectory@1s.x': 100,
      'trajectory@1s.y': 100,
      'trajectory@1s.heading': 100,
    },
  });
  const zeroWeightIndex = buildWith(0);
  const positiveWeightIndex = buildWith(1);
  const query = turnClip().frames[7];
  const zeroWeightResults = queryMotionIndex(zeroWeightIndex, query, { topK: zeroWeightIndex.size });
  const positiveWeightResults = queryMotionIndex(positiveWeightIndex, query, { topK: positiveWeightIndex.size });

  assert.deepEqual(
    zeroWeightResults.map(({ clipId, frameIndex }) => ({ clipId, frameIndex })),
    queryMotionIndex(
      buildMotionIndex(clips, { weights: { pose: 1, motion: 1, trajectory: 0 } }),
      query,
      { topK: zeroWeightIndex.size },
    ).map(({ clipId, frameIndex }) => ({ clipId, frameIndex })),
  );

  let anyDifference = false;
  for (const candidate of zeroWeightResults) {
    const match = positiveWeightResults.find((other) => other.clipId === candidate.clipId && other.frameIndex === candidate.frameIndex);
    assert.ok(match);
    if (Math.abs(candidate.cost - match.cost) > EPSILON) anyDifference = true;
  }
  assert.ok(anyDifference);
});

test('loop seam: a continued-period query maps to the seam frame without skipping frames', () => {
  const loop = loopWalkClip();
  const index = buildMotionIndex([loop]);
  const frameCount = loop.frames.length;

  const seamFrame = structuredClone(loop.frames[0]);
  seamFrame.time = frameCount * FRAME_INTERVAL;
  seamFrame.trajectory = seamFrame.trajectory.map((point) => ({
    ...point,
    t: point.t + frameCount * FRAME_INTERVAL,
  }));

  const candidates = queryMotionIndex(index, seamFrame, { topK: 3 });
  assert.equal(candidates[0].clipId, loop.id);
  assert.equal(circularFrameDistance(candidates[0].frameIndex, 0, frameCount), 0);
  assert.ok(candidates[0].cost < 1e-8);

  const midFrame = structuredClone(loop.frames[10]);
  midFrame.time = loop.frames[10].time + frameCount * FRAME_INTERVAL;
  midFrame.trajectory = midFrame.trajectory.map((point) => ({
    ...point,
    t: point.t + frameCount * FRAME_INTERVAL,
  }));
  const midCandidates = queryMotionIndex(index, midFrame, { topK: 1 });
  assert.equal(circularFrameDistance(midCandidates[0].frameIndex, 10, frameCount), 0);
});

test('circularFrameDistance wraps across the seam', () => {
  assert.equal(circularFrameDistance(19, 0, 20), 1);
  assert.equal(circularFrameDistance(0, 19, 20), 1);
  assert.equal(circularFrameDistance(5, 15, 20), 10);
  assert.equal(circularFrameDistance(2, 2, 20), 0);
});

test('repeated queries return identical stable ordering', () => {
  const index = buildMotionIndex(buildFixture());
  const query = turnClip().frames[9];
  const first = queryMotionIndex(index, query, { topK: 15 });
  const second = queryMotionIndex(index, structuredClone(query), { topK: 15 });
  assert.deepEqual(second, first);

  const signature = (candidate) => `${candidate.clipId}:${candidate.frameIndex}`;
  assert.deepEqual(second.map(signature), first.map(signature));
});

test('equal-cost ties break by clip id then frame index', () => {
  const index = buildMotionIndex([straightClip('aaa'), straightClip('bbb')]);
  const query = straightClip().frames[5];
  const candidates = queryMotionIndex(index, query, { topK: 10 });

  const topTwo = candidates.slice(0, 2);
  assert.ok(Math.abs(topTwo[0].cost - topTwo[1].cost) < EPSILON);
  assert.equal(topTwo[0].clipId, 'aaa');
  assert.equal(topTwo[1].clipId, 'bbb');
});

test('missing foot channels stay masked and are never filled with averages', () => {
  const sparse = straightClip('sparse-straight', { omitRightFoot: true });
  const index = buildMotionIndex([sparse]);
  const candidates = queryMotionIndex(index, sparse.frames[6], { topK: 3 });

  assert.equal(candidates[0].clipId, 'sparse-straight');
  assert.ok(Number.isFinite(candidates[0].cost));
  assert.ok(!Number.isNaN(candidates[0].cost));
  assert.ok(!candidates[0].missingGroups.includes('pose'));

  const features = index.clips[0].features[6];
  assert.deepEqual(features.pose.mask, [true, true, false, false]);
  assert.deepEqual(features.pose.values.slice(2), [0, 0]);
});

test('a clip with no foot data reports the pose group as missing', () => {
  const noFeet = straightClip('no-feet', { omitFeet: true });
  const index = buildMotionIndex([noFeet]);
  const candidates = queryMotionIndex(index, noFeet.frames[3], { topK: 1 });
  assert.deepEqual(candidates[0].missingGroups, ['pose']);
  assert.equal(candidates[0].costs.pose, null);
  assert.ok(candidates[0].costs.motion !== null);
  assert.ok(candidates[0].costs.trajectory !== null);
});

test('zero velocity frames are valid and stop-clip zero-speed frames match each other', () => {
  const stop = stopClip();
  const index = buildMotionIndex([stop]);
  const lastFrame = stop.frames[stop.frames.length - 1];
  assert.equal(lastFrame.velocity.x, 0);
  assert.equal(lastFrame.velocity.y, 0);

  const candidates = queryMotionIndex(index, lastFrame, { topK: 3 });
  assert.ok(candidates.length > 0);
  assert.ok(candidates.every((candidate) => Number.isFinite(candidate.cost)));
  assert.equal(candidates[0].frameIndex, stop.frames.length - 1);
});

test('maxCost filters every candidate and returns an empty list', () => {
  const index = buildMotionIndex(buildFixture());
  const straight = straightClip();
  const candidates = queryMotionIndex(index, straight.frames[0], {
    maxCost: 0,
    excludedClipIds: ['straight'],
  });
  assert.deepEqual(candidates, []);
});

test('a clip without trajectory data still matches on pose and motion groups', () => {
  const noTrajectory = straightClip('no-trajectory', { omitTrajectory: true });
  const index = buildMotionIndex([noTrajectory]);
  const candidates = queryMotionIndex(index, noTrajectory.frames[5], { topK: 1 });
  assert.deepEqual(candidates[0].missingGroups, ['trajectory']);
  assert.ok(candidates[0].costs.pose !== null);
  assert.ok(candidates[0].costs.motion !== null);
  assert.ok(Number.isFinite(candidates[0].cost));
});

test('a single-frame clip works even though acceleration cannot be derived', () => {
  const singleFrameClip = {
    id: 'single-frame',
    loop: false,
    frames: [{
      time: 0,
      root: { x: 0, y: 0, heading: 0 },
      velocity: { x: 0.5, y: 0 },
      feet: {
        left: { x: 0, y: 0.1 },
        right: { x: 0.1, y: -0.1 },
      },
      trajectory: [],
    }],
  };
  const index = buildMotionIndex([singleFrameClip]);
  const queryFrame = {
    time: 2,
    root: { x: 5, y: 5, heading: 1.2 },
    velocity: { x: -0.5, y: 0.3 },
    feet: { left: null, right: null },
    trajectory: [],
  };
  const candidates = queryMotionIndex(index, queryFrame);
  assert.equal(candidates.length, 1);
  assert.ok(Number.isFinite(candidates[0].cost));
  assert.deepEqual(candidates[0].missingGroups, ['pose', 'trajectory']);
  assert.equal(candidates[0].costs.motion !== null, true);
  assert.deepEqual(index.clips[0].features[0].motion.mask, [true, true, false, false]);
});

test('missing values stay masked placeholders instead of inferred averages', () => {
  const sparse = straightClip('sparse-mask', { omitRightFoot: true });
  const index = buildMotionIndex([sparse]);
  const pose = index.clips[0].features[4].pose;
  assert.deepEqual(pose.mask, [true, true, false, false]);
  assert.deepEqual(pose.values.slice(2), [0, 0]);

  const singleFrame = buildMotionIndex([{
    id: 'lonely',
    frames: [{
      time: 0,
      root: { x: 0, y: 0, heading: 0 },
      velocity: { x: 0.2, y: 0 },
      trajectory: [],
    }],
  }]);
  const motion = singleFrame.clips[0].features[0].motion;
  assert.deepEqual(motion.mask, [true, true, false, false]);
  assert.deepEqual(motion.values.slice(2), [0, 0]);
});

test('feature scales are configurable and rescale the cost deterministically', () => {
  const index = buildMotionIndex(buildFixture(), {
    scales: { 'velocity.x': 2, 'velocity.y': 2 },
  });
  const candidates = queryMotionIndex(index, turnClip().frames[5], { topK: 8 });
  assert.ok(candidates.every((candidate) => Number.isFinite(candidate.cost)));
  const again = queryMotionIndex(index, turnClip().frames[5], { topK: 8 });
  assert.deepEqual(again, candidates);
});

test('invalid configuration is rejected', () => {
  assert.throws(() => buildMotionIndex(buildFixture(), { weights: { pose: -1 } }), /pose/);
  assert.throws(() => buildMotionIndex(buildFixture(), { scales: { 'velocity.x': 0 } }), /scale/);
  assert.throws(() => buildMotionIndex(buildFixture(), { horizons: [0] }), /horizons/);
  assert.throws(() => buildMotionIndex(buildFixture(), { scales: { bogus: 1 } }), /Unknown scale/);
  assert.throws(() => buildMotionIndex([{ id: 'x', frames: [] }]), /at least one frame/);
});
