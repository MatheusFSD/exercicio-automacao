import math
import sys
import xml.etree.ElementTree as ET


def perto(atual, esperado, contexto):
    if not math.isfinite(atual) or not math.isclose(
        atual, esperado, rel_tol=1e-8, abs_tol=1e-10
    ):
        raise ValueError(f"{contexto}: esperado {esperado}, recebido {atual}")


def validar(caminho):
    robot = ET.parse(caminho).getroot()
    if any(element.tag.startswith("{http://www.ros.org/wiki/xacro}")
           for element in robot.iter()):
        raise ValueError("Expanda o Xacro antes de executar este teste.")


    referencias = {
        "base_link": (5.0, (0, 0, 0.1), (1/12, 1/6, 13/60)),
        "right_wheel_link": (1.0, (0, 0, 0), (13/4800, 0.005, 13/4800)),
        "left_wheel_link": (1.0, (0, 0, 0), (13/4800, 0.005, 13/4800)),
        "caster_wheel_link": (0.5, (0, 0, 0), (0.0005, 0.0005, 0.0005)),
    }
    total = 0.0
    for nome, (massa, centro, diagonal) in referencias.items():
        link = robot.find(f"link[@name='{nome}']")
        if link is None or link.find("inertial") is None:
            raise ValueError(f"{nome}: link ou inertial ausente")
        if len(link.findall("inertial")) != 1:
            raise ValueError(f"{nome}: deve ter exatamente um inertial")
        inertial = link.find("inertial")
        perto(float(inertial.find("mass").get("value")), massa, f"{nome}/mass")
        origin = inertial.find("origin")
        for atual, esperado in zip(map(float, origin.get("xyz").split()), centro):
            perto(atual, esperado, f"{nome}/centro de massa")
        roll, pitch, yaw = map(float, origin.get("rpy").split())
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        rotacao = (
            (cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr),
            (sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr),
            (-sp, cp*sr, cp*cr),
        )
        tensor = inertial.find("inertia")
        a, b, c, d, e, f = [float(tensor.get(k))
                            for k in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")]
        matriz = ((a, b, c), (b, d, e), (c, e, f))
        for i in range(3):
            for j in range(3):
                valor = sum(rotacao[i][k] * matriz[k][l] * rotacao[j][l]
                            for k in range(3) for l in range(3))
                perto(valor, diagonal[i] if i == j else 0, f"{nome}/I[{i},{j}]")
        total += massa
        print(f"OK {nome}: massa={massa:g} kg; I_link={diagonal} kg*m^2")

    footprint = robot.find("link[@name='base_footprint']")
    if footprint is None or footprint.find("inertial") is not None:
        raise ValueError("base_footprint deve ser um referencial virtual sem inertial")
    perto(total, 7.5, "massa total")
    print("OK: 4 links fisicos; massa total = 7.5 kg; centros e tensores corretos.")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit("Uso: python3 validar_inercia.py /caminho/my_robot.urdf")
    try:
        validar(sys.argv[1])
    except (ValueError, ET.ParseError, OSError, AttributeError, TypeError) as erro:
        sys.exit(f"FALHOU: {erro}")
