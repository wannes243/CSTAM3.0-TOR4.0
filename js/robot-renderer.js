export const RobotRenderer = (() => {
  function draw(ctx, robot, worldToCanvas) {
    const [x, y] = worldToCanvas(robot.x, robot.y);

    ctx.save();
    ctx.translate(x, y);
    ctx.rotate(-robot.yaw);

    ctx.fillStyle = "#38f28a";
    ctx.beginPath();
    ctx.moveTo(15, 0);
    ctx.lineTo(-10, -8);
    ctx.lineTo(-10, 8);
    ctx.closePath();
    ctx.fill();

    ctx.strokeStyle = "#ffffff";
    ctx.lineWidth = 1;
    ctx.stroke();
    ctx.restore();
  }

  return { draw };
})();
