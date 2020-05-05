"""
@File    : excavation_force.py
@Time    : Feb 05, 2020
@Author  : Chunyu Zhang
@Email   : chyzh@ucdavis.edu

This file uses four models, Zeng Model, Swick and Perumpral Model, Gill and Vanden Berg Model and McKyes to calculate
Excavation Force of lunar regolith. The components of the excavation force can also be calculated.
The parameters are defined in config/param_moon.json.

Noted that the result of Zeng Model has not been completely verified.

Reference:
    [1] Zeng, X., Burnoski, L., Agui, J., & Wilkinson, A. (2007, January). Calculation of excavation force for
        ISRU on lunar surface. In 45th AIAA aerospace sciences meeting and exhibit (p. 1474).
    [2] Wilkinson, A., & DeGennaro, A. (2007). Digging and pushing lunar regolith: Classical soil mechanics and
        the forces needed for excavation and traction. Journal of Terramechanics, 44(2), 133-152.
"""
from math import *


class ExcavationForce(object):
    def __init__(self, param):
        self.w = param["tool_width"]
        self.l = param["tool_length"]
        self.d = param["tool_depth"]
        self.l_s = param["side_length"]
        self.s = param["side_thickness"]
        self.alpha_b = param["blunt_edge_angle"]
        self.e_b = param["blunt_edge_thickness"]
        self.g = param["moon_gravity"]
        self.gamma = param["soil_specific_mass"]
        self.q = param["surcharge_mass"]
        self.beta = param["rank_angle"] / 180 * 3.1416
        self.rho = param["shear_plane_failure_angle"] / 180 * 3.1416
        self.v = param["tool_speed"]
        self.K = param["cut_resistance_index"]
        self.c = param["cohesion"]
        self.phi = param["internal_friction_angle"] / 180 * 3.1416
        self.C_a = param["soil_tool_adhesion"]
        self.N0 = param["soil_tool_normal_force"]
        self.delta = param["external_friction_angle"] / 180 * 3.1416

        # Parameters below are only for Zeng Model
        self.a_h = param["horizontal_acceleration"]
        self.a_v = param["vertical_acceleration"]
        self.k0 = param["at_rest_earth_coefficient"]
        self.w_b = param["weight_of_excavation_blade"]
        self.xi = param["soil_blade_friction_angle"] / 180 * 3.1416

    @property
    def tool_depth(self):
        return self.d

    @tool_depth.setter
    def tool_depth(self, value):
        self.d = value

    @property
    def rank_angle(self):
        return self.beta

    @rank_angle.setter
    def rank_angle(self, value):
        self.beta = value

    @property
    def tool_speed(self):
        return self.v

    @tool_speed.setter
    def tool_speed(self, value):
        self.v = value

    def SwickPerumpralModel(self):
        common_factor = self.w * self.d / sin(self.beta + self.phi + self.rho + self.delta)

        def calc_surcharge_force():
            return self.g * self.q * (1 / tan(self.beta) + 1 / tan(self.rho)) * sin(self.phi + self.rho) * common_factor

        def calc_tool_soil_force():
            return -self.C_a * cos(self.beta + self.phi + self.rho) / sin(self.beta) * common_factor

        def calc_depth_force():
            return self.gamma * self.g * self.d / 2 * (1 / tan(self.beta) + 1 / tan(self.rho)) * sin(
                self.phi + self.rho) * common_factor

        def calc_cohesion_force():
            return self.c * cos(self.phi) / sin(self.rho) * common_factor

        def calc_kinetic_force():
            return self.gamma * self.v ** 2 * sin(self.beta) * cos(self.phi) / sin(self.beta + self.rho) * common_factor

        total_force = calc_surcharge_force() + calc_tool_soil_force() + calc_depth_force() + calc_cohesion_force() + calc_kinetic_force()
        horizontal_force = total_force * sin(self.beta + self.delta)
        vertical_force = total_force * cos(self.beta + self.delta)
        return total_force, horizontal_force, vertical_force

    def GillVandenBergModel(self):
        common_factor = self.w * self.d * (sin(self.beta) + self.delta * cos(self.beta)) * (
                sin(self.rho) + self.phi * cos(self.rho)) / (
                                sin(self.rho + self.beta) * (1 - self.phi * self.delta) + cos(
                            self.rho + self.beta) * (self.phi - self.delta))

        def calc_tool_soil_force():
            return self.l * self.g * self.gamma * sin(self.beta + self.rho) / sin(self.rho) * common_factor

        def calc_depth_force():
            return self.g * self.gamma * sin(self.beta + self.rho) / sin(self.rho) * (
                    self.d * cos(self.beta + self.rho) / (2 * sin(self.rho)) + self.d * sin(
                self.beta + self.rho) * tan(self.beta) / (2 * sin(self.rho))) * common_factor

        def calc_cohesion_force():
            return self.c / (sin(self.rho) * (sin(self.rho) + self.phi * cos(self.rho))) * common_factor

        def calc_kinetic_force():
            return self.gamma * self.v ** 2 * sin(self.beta) / (
                    sin(self.beta + self.rho) * (sin(self.rho) + self.phi * cos(self.rho))) * common_factor

        horizontal_force = calc_tool_soil_force() + calc_depth_force() + calc_cohesion_force() + calc_kinetic_force() + self.K * self.w
        vertical_force = horizontal_force / tan(self.beta + self.delta)
        total_force = horizontal_force / sin(self.beta + self.delta)
        return total_force, horizontal_force, vertical_force

    def McKyesModel(self):
        common_factor = self.w * self.d / (
                cos(self.beta + self.delta) + sin(self.beta + self.delta) / tan(self.rho + self.phi))

        def calc_surcharge_force():
            return self.g * self.q * (1 / tan(self.beta) + 1 / tan(self.rho)) * common_factor

        def calc_tool_soil_force():
            return self.C_a * (1 - 1 / tan(self.beta) / tan(self.rho + self.phi)) * common_factor

        def calc_depth_force():
            return self.gamma * self.g * self.d * (1 / tan(self.beta) + 1 / tan(self.rho)) / 2 * common_factor

        def calc_cohesion_force():
            return self.c * (1 + 1 / tan(self.rho) / tan(self.rho + self.phi)) * common_factor

        def calc_kinetic_force():
            return self.gamma * self.v ** 2 * (tan(self.rho) + 1 / tan(self.rho + self.phi)) / (
                    1 + tan(self.rho) / tan(self.beta)) * common_factor

        total_force = calc_surcharge_force() + calc_tool_soil_force() + calc_depth_force() + calc_cohesion_force() + calc_kinetic_force()
        horizontal_force = total_force * sin(self.beta + self.delta)
        vertical_force = total_force * cos(self.beta + self.delta)
        return total_force, horizontal_force, vertical_force

    def ZengModel(self):
        w_b = self.w_b  # Weight of excavation blade
        ww = self.l  # Tool Length
        d = self.d  # Tool Depth
        g = self.g  # Gravity
        a_h = self.a_h  # Horizontal Acceleration
        a_v = self.a_v  # Vertical Acceleration
        q = self.q  # Soil Surcharge
        rho = self.gamma  # Soil Density
        c = self.c  # Soil Cohesion
        alpha = self.beta  # Rank Angle
        phi = self.phi  # Internal Friction Angle
        delta = self.xi  # Soil-Blade Friction Angle
        k0 = self.k0  # At Rest Earth Coefficient

        gamma = rho * g  # unit weight of soil

        def calc_total_excavation_force():
            t_x, t_y = calc_component_excavation_force()
            t = sqrt(t_x ** 2 + t_y ** 2)
            return t

        def calc_component_excavation_force():
            """
            Zeng assumed the failure area is the same as the failure wedge in front of the blade and the direction of the
            movement the same as the inclination angle of the wedge. that's beta = alpha_p
            """
            f_blade = calc_blade_friction_force()
            f_side = calc_side_friction_force()
            p = calc_passive_earth_pressure()
            beta = calc_failure_surface_inclination_angle()
            t_x = -f_blade * sin(alpha) + p * cos(alpha - delta) + f_side * cos(beta) + (
                    w_b / g) * a_h
            t_y = f_blade * cos(alpha) + p * sin(alpha - delta) + f_side * sin(beta) + (
                    w_b / g) * a_v + w_b
            return t_x, t_y

        def calc_blade_friction_force():
            """
            Zeng ignored the blade frictional force due to cohesion of the soil because cohesion of the lunar soil is very small,
            but while the soil of the earth is under consideration, the blade frictional force should not be ignored.
            """
            # return c_a * d * w
            return 0

        def calc_passive_earth_pressure():
            k_pe = calc_passive_pressure_coefficient()
            p = 0.5 * k_pe * (
                    1 + a_v / g) * gamma * d ** 2 * ww + 2 * c * d * ww * sqrt(
                k_pe) + k_pe * q * d * ww
            return p

        def calc_passive_pressure_coefficient():
            """
            For calculation of Pp, the widely used Mononobe-Okabe theory to calculate dynamic earth pressure on a
            retaining wall during earthquake is modified for this application.
            However, in Zeng's paper, this equation was cited incorrectly.
            """
            zie = calc_acceleration_inclination_angle()
            numerator = cos(phi + alpha + zie) ** 2
            denominator = cos(zie) * cos(alpha) ** 2 * cos(delta - alpha - zie) * (
                    1 - sqrt(sin(delta + phi) * sin(phi + zie) / (cos(delta - alpha - zie) * cos(alpha)))) ** 2
            k_pe = numerator / denominator
            return k_pe

        def calc_acceleration_inclination_angle():
            zie = atan(a_h / g + a_v)
            return zie

        def calc_side_friction_force():
            alpha_p = calc_failure_surface_inclination_angle()
            l_w = d * (tan(alpha) + 1 / tan(alpha_p))
            f_side = l_w * (c * d + k0 * q * d * tan(phi) + k0 * gamma * tan(
                phi) * (d ** 2) / 3)
            return f_side

        def calc_failure_surface_inclination_angle():
            zie = calc_acceleration_inclination_angle()
            c_3e = calc_c3e()
            c_4e = calc_c4e()
            alpha_p = -zie - phi + atan((tan(phi - zie) + c_3e) / c_4e)
            return alpha_p

        def calc_c3e():
            zie = calc_acceleration_inclination_angle()
            c_3e = sqrt(
                tan(phi + zie) * (tan(phi + zie) + 1 / tan(phi + alpha + zie)) * (
                        1 + tan(delta - zie - alpha) / tan(phi + alpha + zie)))
            return c_3e

        def calc_c4e():
            zie = calc_acceleration_inclination_angle()
            c_4e = 1 + tan(delta - zie - alpha) * (
                    tan(phi + zie) + 1 / tan(phi + alpha + zie))
            return c_4e

        t = calc_total_excavation_force()
        tx, ty = calc_component_excavation_force()

        return t, tx, ty

