function tau = invdyn( robot, q, qd, qdd, grav_accn )
% INVDYN  Calculate robot inverse dynamics.
% invdyn(robot,q,qd,qdd) calculates the inverse dynamics of a robot using
% the recursive Newton-Euler algorithm, evaluated in link coordinates.
% Gravity is simulated by a fictitious base acceleration of [0,0,9.81] m/s^2
% in base coordinates.  This can be overridden by supplying a 3D vector as
% an optional fifth argument.

if nargin == 4
  grav_accn = [0;0;0;0;0;9.81];		% default gravity
else
  grav_accn = [0;0;0;grav_accn(1);grav_accn(2);grav_accn(3)];
end

for i = 1:robot.NB
  if robot.pitch(i) == 0		% revolute joint
    Xj = Xrotz(q(i));
    S{i} = [0;0;1;0;0;0];
  else robot.pitch(i) == inf		% prismatic joint
    Xj = Xtrans([0 0 q(i)]);
    S{i} = [0;0;0;0;0;1];
  end
  Xup{i} = Xj * robot.Xlink{i};
end

for i = 1:robot.NB
  if robot.parent(i) == 0
    v{i} = S{i} * qd(i);
    a{i} = Xup{i}*grav_accn + S{i}*qdd(i);
  else
    v{i} = Xup{i}*v{robot.parent(i)} + S{i}*qd(i);
    a{i} = Xup{i}*a{robot.parent(i)} + S{i}*qdd(i) + crossM(v{i})*S{i}*qd(i);
  end
  f{i} = robot.Ilink{i}*a{i} + crossF(v{i})*robot.Ilink{i}*v{i};
end

for i = robot.NB:-1:1
  tau(i,1) = S{i}' * f{i};
  if robot.parent(i) ~= 0
    f{robot.parent(i)} = f{robot.parent(i)} + Xup{i}'*f{i};
  end
end
