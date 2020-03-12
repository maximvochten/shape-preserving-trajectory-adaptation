#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Thu Sep  5 13:08:30 2019

@author: Zeno Gillis, Victor van Wymeersch, Maxim Vochten

Helper functions for invariant calculations:
    - integrators
"""

import casadi as cas
import numpy as np

def rodriguez_rot_form(omega,h):
    """Return a rotation matrix which is the result of rotating with omega over time interval h"""
    omega_norm = cas.norm_2(omega)
    #return np.eye(3) + sin(omega_norm*h)/omega_norm*skew(omega) + (1-cos(omega_norm*h))/omega_norm**2 * mtimes(skew(omega),skew(omega))
    return cas.SX.eye(3) + cas.sin(omega_norm*h)/omega_norm*cas.skew(omega) + (1-cas.cos(omega_norm*h))/omega_norm**2 * cas.mtimes(cas.skew(omega),cas.skew(omega))

#def rodriguez_rot_form3(deltatheta):
#    theta = deltatheta[0,1]
#    return cas.SX.eye(3) + cas.mtimes(cas.sin(theta),K) + (1-cas.cos(theta))*cas.mtimes(K,K)


def geo_integrator(R_t, R_r, R_obj, p_obj, u, h):
    """Integrate invariants over interval h starting from a current state (object pose + moving frames)"""
    # Define a geometric integrator for eFSI,
    # (meaning rigid-body motion is perfectly integrated assuming constant invariants)
    i1 = u[0]
    i2 = u[1]
    i3 = u[2]
    i4 = u[3]
    i5 = u[4]
    i6 = u[5]

    omega_t = cas.vertcat(i6,i5,0)
    omega_r = cas.vertcat(i3,i2,0)
    omega_o = cas.mtimes(R_r, cas.vertcat(i1,0,0))

    #translation
    deltaR_t = rodriguez_rot_form(omega_t,h)
    R_t_plus1 = cas.mtimes(R_t,deltaR_t)
    p_obj_plus1 = cas.mtimes(R_t, cas.vertcat(i4,0,0))*h + p_obj

    #rotation
    deltaR_r = rodriguez_rot_form(omega_r,h)
    R_r_plus1 = cas.mtimes(R_r,deltaR_r)

    deltaR_o = rodriguez_rot_form(omega_o,h)
    R_obj_plus1 = cas.mtimes(deltaR_o,R_obj)

    return (R_t_plus1, R_r_plus1, R_obj_plus1, p_obj_plus1)

def rodriguez_rot_form2(omega,h):
    """Return a rotation matrix which is the result of rotating with omega over time interval h"""
    omega_norm = cas.norm_2(omega)
    #return np.eye(3) + sin(omega_norm*h)/omega_norm*skew(omega) + (1-cos(omega_norm*h))/omega_norm**2 * mtimes(skew(omega),skew(omega))
    return np.eye(3) + cas.sin(omega_norm*h)/omega_norm*cas.skew(omega) + (1-cas.cos(omega_norm*h))/omega_norm**2 * cas.mtimes(cas.skew(omega),cas.skew(omega))

def geo_integrator_eFSI(R_t, R_r, R_obj, p_obj, u, h):
    """Integrate invariants over interval h starting from a current state (object pose + moving frames)"""
    #SAME ONE BUT THIS ONE USES ACTUAL VALUES INSTEAD OF CASADI VARIABLES
    # Define a geometric integrator for eFSI,
    # (meaning rigid-body motion is perfectly integrated assuming constant invariants)
    i1 = u[0]
    i2 = u[1]
    i3 = u[2]
    i4 = u[3]
    i5 = u[4]
    i6 = u[5]

    omega_t = np.array([[i6],[i5],[0]])
    omega_r = np.array([[i3],[i2],[0]])
    omega_o = cas.mtimes(R_r, np.array([[i1],[0],[0]]))

    #translation
    deltaR_t = rodriguez_rot_form2(omega_t,h)
    R_t_plus1 = cas.mtimes(R_t,deltaR_t)
    p_obj_plus1 = cas.mtimes(R_t, np.array([[i4],[0],[0]]))*h + p_obj

    #rotation
    deltaR_r = rodriguez_rot_form2(omega_r,h)
    R_r_plus1 = cas.mtimes(R_r,deltaR_r)

    deltaR_o = rodriguez_rot_form2(omega_o,h)
    R_obj_plus1 = cas.mtimes(deltaR_o,R_obj)

    return (R_t_plus1, R_r_plus1, R_obj_plus1, p_obj_plus1)

def offset_integrator(invariants, h):
    """Return offset in object position and rotation by reconstructing the invariant signature"""
    R_t_0 = np.eye(3)
    R_r_0 = np.eye(3)
    R_offset = np.eye(3)
    p_offset = np.array([[0],[0],[0]])
    for i in range(np.shape((invariants))[1]):
        (R_t_0, R_r_0, R_offset, p_offset) \
            = geo_integrator_eFSI(R_t_0, R_r_0, R_offset, p_offset, invariants[:,i], h)
    return (R_offset, p_offset)
