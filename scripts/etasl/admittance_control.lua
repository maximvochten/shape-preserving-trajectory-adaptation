require("context")
require("geometric")

-- ======== Pre-processing ======== --
deadzon = true
-- Frames
tf = task_frame

-- Processing wrench
-- Force inputs
Fx_raw = ctx:createInputChannelScalar("Fx",0.0)
Fy_raw = ctx:createInputChannelScalar("Fy",0.0)
Fz_raw = ctx:createInputChannelScalar("Fz",0.0)

-- Torque inputs
Tx_raw = ctx:createInputChannelScalar("Tx",0.0)
Ty_raw = ctx:createInputChannelScalar("Ty",0.0)
Tz_raw = ctx:createInputChannelScalar("Tz",0.0)

if deadzone then
th_f=constant(3)
th_t=constant(0.5)

Fx_lc  = utils_ts.dead_zone(Fx_raw,th_f)
Fy_lc  = utils_ts.dead_zone(Fy_raw,th_f)
Fz_lc  = utils_ts.dead_zone(Fz_raw,th_f)
Tx_lc  = utils_ts.dead_zone(Tx_raw,th_t)
Ty_lc  = utils_ts.dead_zone(Ty_raw,th_t)
Tz_lc  = utils_ts.dead_zone(Tz_raw,th_t)

-- make wrench expression from deadzoned forces and torques
wr_lc = wrench(vector(Fx_lc,Fy_lc,Fz_lc),vector(Tx_lc,Ty_lc,Tz_lc))
-- allign wrench directions with the ones from robot base frame
wr_lc  = transform(rotation(tf),wr_lc)

else
-- make wrench expression from non-deadzoned forces and torques
wr_lc = wrench(vector(Fx_raw,Fy_raw,Fz_raw),vector(Tx_raw,Ty_raw,Tz_raw))
-- allign wrench directions with the ones from robot base frame
wr_lc  = transform(rotation(tf),wr_lc)

end
-- get the transformed forces and torques
Fx = coord_x(force(wr_lc))
Fy = coord_y(force(wr_lc))
Fz = coord_z(force(wr_lc))

Tx = coord_x(torque(wr_lc))
Ty = coord_y(torque(wr_lc))
Tz = coord_z(torque(wr_lc))

-- Stiffness values
kfx = constant(70)
kfy = constant(70)
kfz = constant(70)

ktx = constant(3)
kty = constant(3)
ktz = constant(3)

-- ======== Constraints ======== --
-- Allowing force interaction
--      Currently, only translation admittance implemented; can be extended to rotational admittance in a similar manner
Constraint {
        context=ctx,
        name="F_x",
        model = coord_x(origin(tf)),
        meas = -Fx/kfx,
        target = 0.0,
        K        = 1,
        weight = 1,
        priority = 2
}

Constraint {
        context=ctx,
        name="F_y",
        model = coord_y(origin(tf)),
        meas = -Fy/kfy,
        target = 0.0,
        K        = 1,
        weight = 1,
        priority = 2
}

Constraint {
        context=ctx,
        name="F_z",
        model = coord_z(origin(tf)),
        meas = -Fz/kfz,
        target =0,
        K        = 1,
        weight = 1,
        priority = 2
}
