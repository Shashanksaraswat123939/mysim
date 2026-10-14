import math, os, re, struct, sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, FancyArrowPatch

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
OUT = os.path.join(ROOT, "docs", "drawings")
os.makedirs(OUT, exist_ok=True)

def cfg(name):
    s = open(os.path.join(ROOT, "firmware/LineFollower/config.h")).read()
    return float(re.search(r"constexpr (?:float|uint8_t) %s = ([0-9.eE+-]+)f?;" % name, s).group(1))

G = {k: cfg(k) for k in ["kWheelBase", "kSensorCount", "kSensorSpacing", "kSensorForward", "kLineWidth",
     "kWheelRadius", "kWheelWidth", "kBodyLength", "kBodyHeight", "kBodyClearance", "kRearRowDistance",
     "kControlPeriod", "kLostTimeout", "kCurveTau", "kDoubleIrLatAccel", "kDoubleIrVMax"]}
BODY_H = os.path.join(ROOT, "firmware", "LineFollower", "body.h")
body_inertia = float(re.search(r"kBodyYawInertia = ([0-9.eE+-]+)f;", open(BODY_H).read()).group(1))
w = open(os.path.join(ROOT, "sim/world.h")).read()
tup = re.search(r"kRealRobot = \{(.*?)\};", w, re.S).group(1)
names = ["mass", "yawInertia", "wheelRadius", "friction", "rollingResistance", "stallForce", "freeSpeed",
         "cornerStiffness", "leftGain", "rightGain", "sensorNoise", "startX", "startY", "startHeading"]
vals = [t.strip().replace("config::kWheelRadius", str(G["kWheelRadius"])).replace("kBodyYawInertia", repr(body_inertia)).rstrip("f") for t in tup.replace("\n", " ").split(",")]
P = dict(zip(names, [float(v) for v in vals]))
mm = lambda v: v * 1000.0

wb, ww, wr = G["kWheelBase"], G["kWheelWidth"], G["kWheelRadius"]
bl, bh, clr = G["kBodyLength"], G["kBodyHeight"], G["kBodyClearance"]
fwd, rear, sp, n = G["kSensorForward"], G["kRearRowDistance"], G["kSensorSpacing"], int(G["kSensorCount"])
bw = wb - ww - 0.004
zt = clr + bh
barW = (n - 1) * sp + 0.012

parts = [
    ("chassis", (-bl/2, bl/2, -bw/2, bw/2, clr, zt)),
    ("card", (-0.029, 0.029, -0.029, 0.029, zt, zt + 0.0016)),
    ("mcu", (-0.004, 0.012, -0.008, 0.008, zt + 0.0016, zt + 0.0046)),
    ("driver", (0.014, 0.024, -0.016, -0.006, zt + 0.0016, zt + 0.0036)),
    ("battery", (-0.024, -0.006, 0.010, 0.026, zt + 0.0016, zt + 0.0126)),
    ("front_board", (fwd - 0.004, fwd + 0.004, -barW/2, barW/2, 0.004, 0.0085)),
    ("front_arm", (bl/2 - 0.001, fwd - 0.004, -0.010, 0.010, 0.010, 0.0135)),
    ("rear_board", (-rear - 0.004, -rear + 0.004, -barW/2, barW/2, 0.004, 0.0085)),
    ("rear_arm", (-rear + 0.004, -bl/2 + 0.001, -0.010, 0.010, 0.010, 0.0135)),
    ("caster_front", (0.020, 0.028, -0.004, 0.004, 0.0, clr)),
    ("caster_rear", (-0.028, -0.020, -0.004, 0.004, 0.0, clr)),
]

def box_tris(b):
    x0, x1, y0, y1, z0, z1 = b
    v = [(x0,y0,z0),(x1,y0,z0),(x1,y1,z0),(x0,y1,z0),(x0,y0,z1),(x1,y0,z1),(x1,y1,z1),(x0,y1,z1)]
    q = [(0,3,2,1),(4,5,6,7),(0,1,5,4),(1,2,6,5),(2,3,7,6),(3,0,4,7)]
    out = []
    for a, b2, c, d in q:
        out += [(v[a], v[b2], v[c]), (v[a], v[c], v[d])]
    return out

def cyl_tris(yc, hw, N=32):
    out = []
    pt = lambda a, y: (wr * math.cos(a), y, wr + wr * math.sin(a))
    for i in range(N):
        a0, a1 = 2*math.pi*i/N, 2*math.pi*(i+1)/N
        out += [(pt(a0, yc-hw), pt(a1, yc-hw), pt(a1, yc+hw)), (pt(a0, yc-hw), pt(a1, yc+hw), pt(a0, yc+hw))]
        out += [((0, yc+hw, wr), pt(a0, yc+hw), pt(a1, yc+hw)), ((0, yc-hw, wr), pt(a1, yc-hw), pt(a0, yc-hw))]
    return out

def write_stl(path):
    tris = [t for _, b in parts for t in box_tris(b)] + cyl_tris(wb/2, ww/2) + cyl_tris(-wb/2, ww/2)
    with open(path, "w") as f:
        f.write("solid line_follower_practice_model\n")
        for t in tris:
            a, b, c = [tuple(mm(x) for x in p) for p in t]
            u = (b[0]-a[0], b[1]-a[1], b[2]-a[2]); v = (c[0]-a[0], c[1]-a[1], c[2]-a[2])
            nx, ny, nz = u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2], u[0]*v[1]-u[1]*v[0]
            l = math.sqrt(nx*nx+ny*ny+nz*nz) or 1
            f.write("facet normal %.5f %.5f %.5f\n outer loop\n" % (nx/l, ny/l, nz/l))
            for p in (a, b, c): f.write("  vertex %.4f %.4f %.4f\n" % p)
            f.write(" endloop\nendfacet\n")
        f.write("endsolid line_follower_practice_model\n")
    return len(tris)

INK, GRAY, LIGHT = "#111111", "#8a8a8a", "#d9d9d9"
plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 9, "axes.linewidth": 0.8})

def sheet(title, number, size=(13.0, 6.2)):
    fig = plt.figure(figsize=size, dpi=170, facecolor="white")
    fig.add_artist(Rectangle((0.008, 0.012), 0.984, 0.976, fill=False, lw=1.4, ec=INK, transform=fig.transFigure))
    x0, y0, bw_, bh_ = 0.58, 0.012, 0.412, 0.085
    fig.add_artist(Rectangle((x0, y0), bw_, bh_, fill=False, lw=1.0, ec=INK, transform=fig.transFigure))
    fig.add_artist(Rectangle((x0 + 0.34, y0), bw_ - 0.34, bh_, fill=False, lw=0.8, ec=INK, transform=fig.transFigure))
    fig.text(x0 + 0.008, y0 + 0.055, title.upper(), fontsize=9.5, weight="bold", va="center")
    fig.text(x0 + 0.008, y0 + 0.022, "Dummy practice model", fontsize=7, va="center", color="#444")
    fig.text(x0 + 0.348, y0 + 0.055, "SHEET", fontsize=6.5, va="center", color="#444")
    fig.text(x0 + 0.348, y0 + 0.022, number, fontsize=11, weight="bold", va="center")
    return fig

def dim(ax, p1, p2, text, off=0.0, horizontal=True, fs=8, tick=0.0):
    """Dimension line between p1 and p2, shifted by off, with arrows and a label."""
    (x1, y1), (x2, y2) = p1, p2
    if horizontal:
        y1 = y2 = y1 + off
        ax.plot([p1[0], p1[0]], [p1[1], y1], color=INK, lw=0.5); ax.plot([p2[0], p2[0]], [p2[1], y2], color=INK, lw=0.5)
    else:
        x1 = x2 = x1 + off
        ax.plot([p1[0], x1], [p1[1], p1[1]], color=INK, lw=0.5); ax.plot([p2[0], x2], [p2[1], p2[1]], color=INK, lw=0.5)
    ax.add_patch(FancyArrowPatch((x1, y1), (x2, y2), arrowstyle="<|-|>", mutation_scale=7, lw=0.7, color=INK, shrinkA=0, shrinkB=0))
    mx, my = (x1 + x2) / 2, (y1 + y2) / 2
    if horizontal:
        ax.text(mx, my, text, ha="center", va="bottom", fontsize=fs, bbox=dict(fc="white", ec="none", pad=0.6))
    else:
        ax.text(mx, my, text, ha="right", va="center", fontsize=fs, rotation=90, bbox=dict(fc="white", ec="none", pad=0.6))

def centre_line(ax, p1, p2):
    ax.plot([p1[0], p2[0]], [p1[1], p2[1]], color=INK, lw=0.5, ls=(0, (10, 2, 1.5, 2)))

def rect(ax, x0, x1, y0, y1, fc="white", lw=1.0, hatch=None):
    ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, fc=fc, ec=INK, lw=lw, hatch=hatch))

def view_axes(fig, rect_, title):
    ax = fig.add_axes(rect_)
    ax.set_aspect("equal"); ax.axis("off")
    ax.set_title(title, fontsize=9, weight="bold", loc="left")
    return ax

def draw_three_view():
    fig = sheet("Orthographic views", "1 of 6")
    L = 1000.0
    ax = view_axes(fig, [0.04, 0.15, 0.36, 0.78], "TOP VIEW")
    ax.set_xlim(-80, 80); ax.set_ylim(-62, 62)
    for sgn in (1, -1):
        rect(ax, mm(sgn*wb/2 - ww/2) if sgn > 0 else mm(sgn*wb/2 - ww/2), mm(sgn*wb/2 + ww/2), -mm(wr), mm(wr), fc="#bdbdbd")
    for nme, b in parts:
        x0, x1, y0, y1, z0, z1 = [mm(v) for v in b]
        fc = {"chassis": "white", "card": "#e0e0e0", "battery": "#bdbdbd", "mcu": "#777", "driver": "#999"}.get(nme, "#d0d0d0" if "board" in nme else "white")
        if nme.startswith("caster"): continue
        rect(ax, y0, y1, x0, x1, fc=fc, lw=0.9)
    for xr in (fwd, -rear):
        for i in range(n):
            y = ((n - 1) / 2 - i) * sp
            ax.add_patch(Circle((mm(y), mm(xr)), 2.2, fc="black", ec="black"))
    centre_line(ax, (0, -64), (0, 64)); centre_line(ax, (-60, 0), (60, 0))
    dim(ax, (-mm(wb/2 + ww/2), -54), (mm(wb/2 + ww/2), -54), "%.0f overall width" % mm(wb + ww), off=0, fs=7.5)
    dim(ax, (-mm(wb/2), -mm(wr)), (mm(wb/2), -mm(wr)), "%.0f wheelbase" % mm(wb), off=-12)
    dim(ax, (mm(wb/2 + ww/2) + 8, 0), (mm(wb/2 + ww/2) + 8, mm(fwd)), "%.0f front row to axle" % mm(fwd), horizontal=False, off=6)
    dim(ax, (mm(wb/2 + ww/2) + 8, 0), (mm(wb/2 + ww/2) + 8, -mm(rear)), "%.0f rear row to axle" % mm(rear), horizontal=False, off=6)
    ax.text(0, mm(fwd) + 6, "%d sensors at %.0f spacing" % (n, mm(sp)), fontsize=7.5, ha="center")
    ax2 = view_axes(fig, [0.42, 0.52, 0.24, 0.41], "FRONT VIEW")
    ax2.set_xlim(-70, 70); ax2.set_ylim(-8, 52)
    ax2.plot([-70, 70], [0, 0], color=INK, lw=1.2)
    for sgn in (1, -1):
        rect(ax2, mm(sgn*wb/2 - ww/2), mm(sgn*wb/2 + ww/2), 0, mm(2*wr), fc="#bdbdbd")
    rect(ax2, -mm(bw/2), mm(bw/2), mm(clr), mm(zt), fc="white")
    rect(ax2, -29, 29, mm(zt), mm(zt) + 1.6, fc="#e0e0e0")
    rect(ax2, -4, 4, mm(zt) + 1.6, mm(zt) + 4.6, fc="#777")
    rect(ax2, -mm(barW/2), mm(barW/2), 4, 8.5, fc="#d0d0d0")
    centre_line(ax2, (0, -6), (0, 50))
    dim(ax2, (mm(wb/2 + ww/2) + 6, 0), (mm(wb/2 + ww/2) + 6, mm(2*wr)), "%.0f wheel dia" % mm(2*wr), horizontal=False, off=8)
    top = mm(zt) + 12.6
    dim(ax2, (-mm(wb/2 + ww/2) - 4, 0), (-mm(wb/2 + ww/2) - 4, top), "%.1f overall height" % top, horizontal=False, off=-4)
    ax3 = view_axes(fig, [0.42, 0.12, 0.24, 0.38], "SIDE VIEW")
    ax3.set_xlim(-60, 60); ax3.set_ylim(-12, 52)
    ax3.plot([-60, 60], [0, 0], color=INK, lw=1.2)
    for nme, b in parts:
        x0, x1, y0, y1, z0, z1 = [mm(v) for v in b]
        rect(ax3, x0, x1, z0, z1, fc={"chassis": "white", "card": "#e0e0e0", "battery": "#bdbdbd", "mcu": "#777", "driver": "#999"}.get(nme, "#d0d0d0"))
    ax3.add_patch(Circle((0, mm(wr)), mm(wr), fc="#bdbdbd", ec=INK, lw=1.0, alpha=0.9)); ax3.add_patch(Circle((0, mm(wr)), 1.2, fc=INK))
    centre_line(ax3, (-56, mm(wr)), (56, mm(wr)))
    dim(ax3, (-mm(rear), -3), (mm(fwd), -3), "%.0f sensor to sensor" % mm(rear + fwd), off=-3.5, fs=7.5)
    dim(ax3, (-mm(bl/2), mm(zt) + 12), (mm(bl/2), mm(zt) + 12), "%.0f body" % mm(bl), off=2, fs=7.5)
    fig.text(0.70, 0.88, "NOTES", fontsize=9, weight="bold")
    for i, t in enumerate([
        "1  All sizes in millimetres.",
        "2  Sizes come from config.h, the same file the",
        "    firmware and the 3D simulator read.",
        "3  The board on top is a dummy line follower",
        "    card. Replace it with the real board.",
        "4  Two sensor rows for Double IR mode.",
        "5  Third angle projection.",
    ]):
        fig.text(0.70, 0.84 - i * 0.04, t, fontsize=8.5)
    fig.savefig(os.path.join(OUT, "01_orthographic_views.png")); plt.close(fig)

MASSES = [
    ("Chassis plate", 0.078, 0.0, 0.0, bl, bw),
    ("Line follower card", 0.018, 0.0, 0.0, 0.058, 0.058),
    ("Battery", 0.070, -0.015, 0.018, 0.018, 0.016),
    ("Microcontroller", 0.010, 0.004, 0.0, 0.016, 0.016),
    ("Motor left", 0.030, 0.0, wb/2 - 0.015, 0.024, 0.012),
    ("Motor right", 0.030, 0.0, -(wb/2 - 0.015), 0.024, 0.012),
    ("Wheel left", 0.018, 0.0, wb/2, 2*wr, ww),
    ("Wheel right", 0.018, 0.0, -wb/2, 2*wr, ww),
    ("Front sensor board", 0.010, fwd, 0.0, 0.008, barW),
    ("Rear sensor board", 0.010, -rear, 0.0, 0.008, barW),
    ("Arms and casters", 0.008, 0.0, 0.0, 0.0, 0.0),
]
def inertia():
    M = sum(m for _, m, *_ in MASSES)
    xc = sum(m * x for _, m, x, *_ in MASSES) / M
    yc = sum(m * y for _, m, _, y, *_ in MASSES) / M
    rows, I = [], 0.0
    for nme, m, x, y, sx, sy in MASSES:
        own = m * (sx*sx + sy*sy) / 12.0
        d2 = (x - xc)**2 + (y - yc)**2
        rows.append((nme, m, x, y, own, m * d2)); I += own + m * d2
    return M, xc, yc, I, rows

def draw_inertia():
    M, xc, yc, I, rows = inertia()
    fig = sheet("Yaw inertia of the robot", "2 of 6")
    ax = view_axes(fig, [0.03, 0.13, 0.36, 0.78], "TOP VIEW WITH PART CENTRES")
    ax.set_xlim(-80, 80); ax.set_ylim(-70, 70)
    for nme, b in parts:
        if nme.startswith("caster"): continue
        x0, x1, y0, y1, *_ = [mm(v) for v in b]
        rect(ax, y0, y1, x0, x1, fc="#f2f2f2", lw=0.7)
    for sgn in (1, -1):
        rect(ax, mm(sgn*wb/2 - ww/2), mm(sgn*wb/2 + ww/2), -mm(wr), mm(wr), fc="#d9d9d9", lw=0.7)
    for nme, m, x, y, *_ in rows:
        ax.add_patch(Circle((mm(y), mm(x)), 1.6, fc=INK))
    ax.add_patch(Circle((mm(yc), mm(xc)), 3.4, fc="white", ec=INK, lw=1.2)); ax.plot([mm(yc)-5, mm(yc)+5], [mm(xc)]*2, color=INK, lw=0.8); ax.plot([mm(yc)]*2, [mm(xc)-5, mm(xc)+5], color=INK, lw=0.8)
    ax.annotate("centre of mass\n(%.1f, %.1f) mm" % (mm(xc), mm(yc)), (mm(yc), mm(xc)), (28, -42), fontsize=8, arrowprops=dict(arrowstyle="-", lw=0.6))
    ax.plot([mm(yc), mm(wb/2)], [mm(xc), 0], color=INK, lw=0.6, ls="--")
    ax.text(mm(wb/4), -4, "d", fontsize=8)
    t = fig.add_axes([0.41, 0.13, 0.57, 0.78]); t.axis("off"); t.set_xlim(0, 1); t.set_ylim(0, 1)
    t.text(0, 1.0, "Method", fontsize=10, weight="bold", va="top")
    t.text(0, 0.94, "Treat each part as a small flat block. Spin about the vertical axis through the centre of mass.", fontsize=8.3, va="top")
    t.text(0, 0.88, r"$I_z=\sum_i\left[\,m_i\,\frac{a_i^2+b_i^2}{12}\;+\;m_i\,d_i^2\right]$     $a_i,b_i$ block sides     $d_i$ distance to the centre of mass", fontsize=9.5, va="top")
    cols = [0.0, 0.34, 0.45, 0.57, 0.69, 0.86]
    hdr = ["Part", "m (g)", "x (mm)", "y (mm)", "own (g mm²)", "m d² (g mm²)"]
    y = 0.76
    for c, h in zip(cols, hdr): t.text(c, y, h, fontsize=7.8, weight="bold")
    t.plot([0, 1], [y - 0.015, y - 0.015], color=INK, lw=0.8)
    for nme, m, x, yy, own, md2 in rows:
        y -= 0.047
        for c, v in zip(cols, [nme, "%.0f" % (m*1000), "%.1f" % mm(x), "%.1f" % mm(yy), "%.0f" % (own*1e9), "%.0f" % (md2*1e9)]):
            t.text(c, y, v, fontsize=7.8)
    y -= 0.04
    t.plot([0, 1], [y + 0.02, y + 0.02], color=INK, lw=0.8)
    t.text(0, y - 0.02, "Total mass %.0f g" % (M*1000), fontsize=8.5, weight="bold")
    t.text(0.45, y - 0.02, r"$I_z=%.2f\times10^{-4}$ kg m²" % (I*1e4), fontsize=10.5, weight="bold")
    fig.text(0.03, 0.062, "The simulator uses this value. It is written to firmware/LineFollower/body.h by cad/make_cad.py.", fontsize=8.6)
    fig.text(0.03, 0.032, "Part masses are assumed, not weighed. Weigh the real parts and rerun cad/make_cad.py.", fontsize=8.1, color="#444")
    fig.savefig(os.path.join(OUT, "02_inertia.png")); plt.close(fig)
    return I

def draw_slip():
    m, mu, g = P["mass"], P["friction"], 9.81
    N = m * g / 2
    fig = sheet("Tyre grip and wheel slip", "3 of 6")
    ax = fig.add_axes([0.07, 0.17, 0.30, 0.68])
    s = [i / 200 * 1.0 for i in range(0, 201)]
    f = [math.tanh(x / 0.2) for x in s]
    ax.plot(s, f, color=INK, lw=1.4)
    ax.axhline(1.0, color=GRAY, lw=0.7, ls="--")
    ax.axvline(0.2, color=GRAY, lw=0.7, ls=":")
    ax.plot([0.2], [math.tanh(1)], "o", color=INK)
    ax.annotate("slip 0.2 gives 76 percent\nof the grip limit", (0.2, math.tanh(1)), (0.36, 0.45), fontsize=8, arrowprops=dict(arrowstyle="-", lw=0.6))
    ax.text(0.98, 1.03, "grip limit  mu N", ha="right", fontsize=8)
    ax.set_xlim(0, 1); ax.set_ylim(0, 1.12)
    ax.set_xlabel("slip ratio  s = (wheel speed - ground speed) / reference speed"); ax.set_ylabel("tyre force / (mu N)")
    for sp_ in ("top", "right"): ax.spines[sp_].set_visible(False)
    ax.grid(color=LIGHT, lw=0.5)
    ax.set_title("FORCE CURVE   F = mu N tanh(s / 0.2)", fontsize=9, weight="bold", loc="left")
    ax2 = view_axes(fig, [0.42, 0.20, 0.22, 0.60], "FRICTION CIRCLE (one wheel)")
    ax2.set_xlim(-1.4, 1.4); ax2.set_ylim(-1.4, 1.4)
    ax2.add_patch(Circle((0, 0), 1, fill=False, ec=INK, lw=1.2))
    ax2.annotate("", (0.0, 0.0), (0.6, 0.0)); 
    for a, b, tx in [((0, 0), (0.62, 0), "F drive"), ((0, 0), (0, 0.78), "F side")]:
        ax2.add_patch(FancyArrowPatch(a, b, arrowstyle="-|>", mutation_scale=9, lw=1.0, color=INK))
    ax2.plot([0.62, 0.62, 0], [0, 0.78, 0.78], color=GRAY, lw=0.6, ls="--")
    ax2.text(0.66, 0.02, "drive", fontsize=8); ax2.text(0.04, 0.82, "sideways", fontsize=8)
    ax2.text(0, -1.25, "Drive and sideways force together\nmust stay inside the circle. Radius = mu N.", ha="center", fontsize=8)
    t = fig.add_axes([0.66, 0.13, 0.32, 0.78]); t.axis("off"); t.set_xlim(0, 1); t.set_ylim(0, 1)
    lines = [
        ("Worked numbers", True),
        ("Mass  m = %.2f kg   Friction  mu = %.1f" % (m, mu), False),
        ("Load on one wheel  N = m g / 2", False),
        ("      = %.2f x 9.81 / 2 = %.2f N" % (m, N), False),
        ("Grip limit per wheel  mu N = %.2f N" % (mu * N), False),
        ("Sideways acceleration limit", False),
        ("      a = mu g = %.1f m/s²" % (mu * g), False),
        ("Double IR plans for %.1f m/s²" % G["kDoubleIrLatAccel"], False),
        ("      that is %.0f%% of the limit" % (100 * G["kDoubleIrLatAccel"] / (mu * g)), False),
        ("Corner speed  v = sqrt(a R)", False),
        ("      R = 0.25 m gives %.2f m/s" % math.sqrt(G["kDoubleIrLatAccel"] * 0.25), False),
        ("Rolling drag  c m g = %.2f N" % (P["rollingResistance"] * m * g), False),
        ("Sideways slide: force = C v_y / 0.2,", False),
        ("capped by the friction circle. C = %.0f N/rad" % P["cornerStiffness"], False),
    ]
    for i, (tx, b) in enumerate(lines):
        t.text(0, 1.0 - i * 0.066, tx, fontsize=9.2 if b else 8.6, weight="bold" if b else "normal", va="top")
    fig.savefig(os.path.join(OUT, "03_slip.png")); plt.close(fig)

def draw_encoder():
    fig = sheet("Wheel encoders and turn rate", "4 of 6")
    ax = view_axes(fig, [0.03, 0.13, 0.30, 0.78], "ENCODER DISC (ILLUSTRATIVE, 20 SLOTS)")
    ax.set_xlim(-30, 30); ax.set_ylim(-30, 30)
    ax.add_patch(Circle((0, 0), 20, fc="#e6e6e6", ec=INK, lw=1.2))
    ax.add_patch(Circle((0, 0), 4, fc="white", ec=INK, lw=1.0))
    for i in range(20):
        a = 2 * math.pi * i / 20
        ax.plot([12 * math.cos(a), 17 * math.cos(a)], [12 * math.sin(a), 17 * math.sin(a)], color=INK, lw=2.2, solid_capstyle="butt")
    ax.add_patch(Rectangle((-2, 21), 4, 5, fc="white", ec=INK)); ax.text(5, 24, "sensor", fontsize=8)
    centre_line(ax, (-27, 0), (27, 0)); centre_line(ax, (0, -27), (0, 27))
    dim(ax, (-20, -22), (20, -22), "wheel dia %.0f mm" % mm(2 * wr), off=-3, fs=8)
    ticks = 20
    v = 0.40
    per_tick = 2 * math.pi * wr / ticks
    t = fig.add_axes([0.35, 0.13, 0.63, 0.78]); t.axis("off"); t.set_xlim(0, 1); t.set_ylim(0, 1)
    vL, vR = 0.35, 0.45
    om = (vR - vL) / wb
    vm = (vR + vL) / 2
    k = om / vm
    L = [
        ("From encoder counts to speed", True),
        ("Distance per count  =  2 pi r / counts per turn  =  %.2f mm      (20 counts is an assumption)" % mm(per_tick), False),
        ("Wheel speed  v = distance per count x counts / time          at %.2f m/s a count arrives every %.1f ms" % (v, 1000 * per_tick / v), False),
        ("The control loop runs every %.0f ms, so speed is counted over a window of several loops." % mm(G["kControlPeriod"]), False),
        ("", False),
        ("From two wheels to turn rate", True),
        ("Body speed          v = (v_R + v_L) / 2", False),
        ("Turn rate           w = (v_R - v_L) / wheelbase", False),
        ("Curvature           k = w / v        radius  R = 1 / k", False),
        ("", False),
        ("Example   v_L = %.2f   v_R = %.2f   wheelbase = %.3f m" % (vL, vR, wb), True),
        ("v = %.2f m/s     w = (%.2f - %.2f) / %.3f = %.2f rad/s" % (vm, vR, vL, wb, om), False),
        ("k = %.2f / %.2f = %.2f 1/m     R = %.0f mm" % (om, vm, k, 1000 / k), False),
        ("", False),
        ("Double IR uses both eyes and encoders", True),
        ("The two sensor rows give the line direction  theta = (y_front - y_rear) / (%.0f + %.0f mm)." % (mm(fwd), mm(rear)), False),
        ("Curvature of the line  k = (d theta / dt + w_encoder) / v, smoothed over %.0f ms." % mm(G["kCurveTau"]), False),
        ("Grip limited speed  v = sqrt(%.1f / |k|), never above %.2f m/s." % (G["kDoubleIrLatAccel"], G["kDoubleIrVMax"]), False),
    ]
    for i, (tx, b) in enumerate(L):
        t.text(0, 1.0 - i * 0.056, tx, fontsize=9.6 if b else 8.6, weight="bold" if b else "normal", va="top")
    fig.savefig(os.path.join(OUT, "04_encoders.png")); plt.close(fig)

def draw_gaps():
    fig = sheet("Dashed lines and gaps", "5 of 6")
    ax = fig.add_axes([0.04, 0.50, 0.92, 0.36]); ax.set_xlim(0, 1.0); ax.set_ylim(-0.1, 0.55); ax.axis("off")
    ax.set_title("DASHED LINE MAP, FIRST SECTION (METRES)", fontsize=9, weight="bold", loc="left")
    pieces = [("tape", 0.40), ("gap", 0.06), ("tape", 0.40), ("arc", 0.0)]
    x = 0.02
    for kind, ln in pieces[:3]:
        if kind == "tape":
            ax.add_patch(Rectangle((x, 0.22), ln, 0.03, fc=INK, ec=INK))
        else:
            ax.plot([x, x + ln], [0.235, 0.235], color=GRAY, lw=0.8, ls=(0, (2, 3)))
            dim(ax, (x, 0.235), (x + ln, 0.235), "gap %.2f m" % ln, off=0.1, fs=8)
        x += ln
    ax.text(0.22, 0.30, "tape 0.40 m", ha="center", fontsize=8); ax.text(0.68, 0.30, "tape 0.40 m", ha="center", fontsize=8)
    ax.add_patch(Rectangle((0.37, 0.15), 0.07, 0.17, fc="none", ec=INK, lw=0.8, ls="--"))
    ax.text(0.405, 0.04, "robot sensor bar\nsees no tape", ha="center", fontsize=7.8)
    t = fig.add_axes([0.04, 0.10, 0.92, 0.34]); t.axis("off"); t.set_xlim(0, 1); t.set_ylim(0, 1)
    gaps = [0.06, 0.10]
    L = [
        ("What the robot does in a gap", True),
        ("The sensors see nothing, so the robot counts how long it has been blind. It gives up after %.1f s." % G["kLostTimeout"], False),
        ("Normal PID holds the last line offset it saw. Full map modes keep following the racing plan, so the map carries them across.", False),
        ("Time to cross a gap  =  gap length / speed", True),
    ]
    for i, (tx, b) in enumerate(L):
        t.text(0, 1.0 - i * 0.14, tx, fontsize=9.6 if b else 8.8, weight="bold" if b else "normal", va="top")
    for j, v in enumerate([0.25, 0.40, 1.00]):
        for i, g_ in enumerate(gaps):
            t.text(0.00 + j * 0.33, 0.40 - i * 0.12, "at %.2f m/s a %.2f m gap takes %.0f ms" % (v, g_, 1000 * g_ / v), fontsize=8.8)
    t.text(0, 0.12, "Longest gap that can be crossed  =  %.1f s x speed.  At 0.40 m/s that is %.0f mm, at 1.00 m/s it is %.0f mm." % (G["kLostTimeout"], 1000 * G["kLostTimeout"] * 0.4, 1000 * G["kLostTimeout"] * 1.0), fontsize=8.8)
    t.text(0, 0.0, "The widest gap in the six maps is 0.10 m, so there is a large margin.", fontsize=8.8)
    fig.savefig(os.path.join(OUT, "05_gaps.png")); plt.close(fig)

def draw_timing():
    fig = sheet("Timestamps and the control loop", "6 of 6")
    ax = fig.add_axes([0.22, 0.24, 0.72, 0.62])
    v = 0.40
    per_tick = 2 * math.pi * wr / 20
    items = [
        ("Control loop period", G["kControlPeriod"], "every 2 ms, 500 times a second"),
        ("Encoder count (20 per turn)", per_tick / v, "one count every %.1f ms" % (1000 * per_tick / v)),
        ("Serial telemetry print", 0.100, "every 100 ms"),
        ("Curvature smoothing (Double IR)", G["kCurveTau"], "%.0f ms time constant" % mm(G["kCurveTau"])),
        ("Sensor look ahead at 0.4 m/s", fwd / v, "%.0f mm ahead is %.0f ms" % (mm(fwd), 1000 * fwd / v)),
        ("Derail rule in the simulator", 0.25, "more than 35 mm off for 250 ms"),
        ("Give up when the line is lost", G["kLostTimeout"], "%.0f ms without tape" % (1000 * G["kLostTimeout"])),
        ("Sensor calibration spin", 2.0, "2 s at start"),
        ("Run time limit", 40.0, "40 s"),
    ]
    items = items[::-1]
    ys = list(range(len(items)))
    ax.barh(ys, [i[1] for i in items], color="#555", height=0.5)
    ax.set_xscale("log"); ax.set_xlim(1e-3, 100)
    ax.set_yticks(ys); ax.set_yticklabels([i[0] for i in items], fontsize=8.6)
    for y, (_, val, note) in zip(ys, items):
        ax.text(val * 1.25, y, note, va="center", fontsize=8.3)
    ax.set_xlabel("seconds (logarithmic scale)")
    for sp_ in ("top", "right"): ax.spines[sp_].set_visible(False)
    ax.grid(axis="x", color=LIGHT, lw=0.5)
    ax.set_title("EVERY TIMESTAMP THE ROBOT RUNS ON", fontsize=9, weight="bold", loc="left")
    fig.text(0.03, 0.05, "The simulator steps physics at the same 2 ms as the control loop.", fontsize=8.6)
    fig.savefig(os.path.join(OUT, "06_timing.png")); plt.close(fig)

if __name__ == "__main__":
    k = write_stl(os.path.join(ROOT, "cad", "line_follower_robot.stl"))
    draw_three_view(); I = draw_inertia(); draw_slip(); draw_encoder(); draw_gaps(); draw_timing()
    with open(BODY_H, "w") as f:
        f.write("#pragma once\n\nconstexpr float kBodyYawInertia = %.6ef;\n" % I)
    M = sum(m for _, m, *_ in MASSES)
    print("stl triangles:", k, " mass %.0f g" % (M * 1000), " Iz %.2e kg m2 (sim %.1e)" % (I, P["yawInertia"]))
