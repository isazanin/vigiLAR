#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
vigiLAR - CDC.6NA  -  Robô Móvel Inteligente de Vigilância Residencial (Pygame)
==============================================================================
Projeto da disciplina Intelligent Mobile Robots (IMR).
Cenário: imagem "Planta_Baixa_Modern_House.jpeg" (deixe na mesma pasta).

O que está sendo demonstrado (e de qual aula vem):
  AULA 01/02  Pose (x, y, theta), cinemática diferencial, odometria, Go-to-Goal (P)
  AULA 03     Sensor LiDAR por raycasting (360 raios, com ruído)
  AULA 05     Exercícios AC-2: converter_cmd_vel (saturação), processar_scan,
              controle_reativo, calcular_orientacao_alvo, máquina de estados (FSM)
  AULA 06/07  Arquitetura estilo ROS 2: nós + tópicos (/scan, /odom, /cmd_vel ...)
  "IA"        A* (planejamento global), mapeamento por grade de ocupação (log-odds),
              replanejamento com obstáculo surpresa e exploração por fronteiras.
  VIGILÂNCIA  Patrulha autônoma, modo noturno, detecção de movimento suspeito por
              LiDAR (varredura medida x varredura esperada da planta) e alertas.

Dependências:  pip install pygame numpy
Execução:      python vigilar.py
"""
import json
import math
import heapq
import os
import random
import threading
import time
import types
import urllib.request
from collections import deque

import numpy as np
import pygame

# ============================================================================
# 1. CONFIGURAÇÕES GERAIS
# ============================================================================
IMG_FILE = "Planta_Baixa_Modern_House.jpeg"
IMG_W, IMG_H = 1242, 710          # tamanho original da planta (pixels)
CELL = 6                          # tamanho da célula das grades (px da planta)
MAPW, MAPH = 1248, 714            # área da simulação (múltiplo de CELL)
GC, GR = MAPW // CELL, MAPH // CELL
PX_PER_M = 100.0                  # escala: 100 px da planta = 1 metro

PANEL_W = 345                     # painel lateral
BOTTOM_H = 92                     # barra inferior (log + ajuda)

# --- Robô (diferencial) ---
ROBOT_R = 0.12                    # raio (m)
WHEELBASE = 0.28                  # distância entre rodas L (m)
MAX_WHEEL = 1.0                   # limite físico de cada roda (m/s)
V_MAX, W_MAX = 0.75, 2.4          # limites de comando
MAX_ACC = 2.2                     # aceleração máx. das rodas (m/s²)
SLIP = (0.998, 1.002)             # patinagem (erro sistemático -> deriva da odometria)

# --- LiDAR ---
LIDAR_RANGE, LIDAR_MIN = 5.0, 0.10
LIDAR_NOISE, LIDAR_DROP = 0.01, 0.015

# --- Controle / FSM ---
KP_ANG, KP_DIST = 2.8, 1.1
TOL_WP, TOL_FINAL = 0.10, 0.12
LOOKAHEAD = 0.22                  # distância da 'cenoura' no segmento (m)
D_CRITICA, D_SAIDA = 0.26, 0.42   # trava de segurança frontal (m)
INFLATE_CELLS = 3.4               # inflação de obstáculos p/ planejar (~0,20 m)

# --- vigiLAR: vigilância ---
NOME_PROJETO = "vigiLAR - CDC.6NA"
V_NOITE = 0.45                    # velocidade máx. da ronda noturna (m/s)
INTRUSO_R, INTRUSO_V = 0.17, 0.35 # pessoa simulada (raio m, velocidade m/s)
ANOM_MARGEM = 0.30                # medido < esperado - margem => algo novo no cenário (m)
MOVE_JANELA, MOVE_DESLOC = 1.5, 0.30   # "está se movendo": desloca >0,30 m em 1,5 s
ALERTA_COOLDOWN = 10.0            # s entre alertas do mesmo alvo
ALERT_LOG = os.environ.get("VIGILAR_LOG", os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                       "alertas_vigilar.log"))
ALERT_WEBHOOK = os.environ.get("VIGILAR_WEBHOOK", "")      # URL opcional (POST JSON)

# ============================================================================
# 2. A PLANTA: paredes, móveis e portas (coordenadas em pixels da imagem)
#    -> Edite aqui se quiser ajustar o mapa!  (x0, y0, x1, y1)
# ============================================================================
PAREDES = [   # retângulos (nome, x0, y0, x1, y1)
    ("parede topo", 0, 0, MAPW, 33), ("parede base", 0, 690, MAPW, MAPH),
    ("parede esq.", 0, 0, 57, MAPH), ("parede dir.", 1218, 0, MAPW, MAPH),
    # divisória sala | quarto/banheiro: CONTÍNUA (a V1 tinha 2 vãos de porta que não existem na planta)
    ("divisória", 720, 33, 735, 690),
    ("parede quarto/banheiro", 735, 447, 1218, 459),
    # faixas de parede visíveis na planta (marcações vermelhas)
    ("parede topo cozinha", 57, 33, 420, 66),
    ("parede base sala", 57, 641, 445, 690), ("parede base entrada", 445, 662, 720, 690),
    ("parede base banheiro", 735, 641, 1218, 690),
    ("parede dir. banheiro", 1170, 459, 1218, 690),
]
PAREDES_POLI = [   # polígonos (nome, [(x, y), ...]) - paredes inclinadas pela perspectiva da imagem
    ("parede esq. interna", [(57, 33), (112, 33), (112, 140), (143, 140), (127, 415),
                             (127, 438), (144, 440), (140, 641), (57, 641)]),
    ("parede dir. quarto", [(1200, 33), (1218, 33), (1218, 447), (1170, 447), (1170, 390),
                            (1100, 390), (1100, 312), (1135, 312), (1135, 150), (1157, 110), (1200, 108)]),
    ("frente do guarda-roupa", [(735, 108), (1200, 108), (1200, 146), (735, 118)]),
]
MOVEIS = [
    # cozinha
    ("ilha+fogão", 197, 33, 476, 185), ("geladeira", 112, 67, 197, 155),
    ("bancada", 420, 33, 720, 172), ("bancada L", 628, 108, 718, 232),
    # jantar
    ("mesa", 138, 305, 325, 418), ("cadeira verm.", 232, 255, 312, 305),
    ("cadeira", 158, 285, 225, 305), ("cadeira", 150, 418, 222, 432),
    ("cadeira", 236, 418, 312, 432), ("banco janela", 98, 288, 138, 435),
    # sala
    ("poltrona", 335, 408, 447, 500), ("sofá", 152, 475, 262, 636),
    ("sofá", 152, 535, 442, 636), ("mesa centro", 285, 490, 376, 536),
    ("aparador", 440, 590, 478, 632), ("degrau", 470, 545, 576, 666),
    ("rack TV", 662, 328, 720, 482), ("prateleira", 622, 478, 720, 498),
    # quarto
    ("guarda-roupa", 735, 33, 1200, 108), ("cama", 790, 218, 987, 416),
    ("criado-mudo", 735, 392, 822, 442), ("baú", 968, 385, 1192, 447),
    ("escrivaninha", 1035, 195, 1122, 292), ("cadeira esc.", 992, 212, 1048, 272),
    # banheiro
    ("pia", 735, 458, 910, 495), ("vaso", 922, 458, 968, 512),
    ("box", 1018, 552, 1168, 668), ("banco", 735, 598, 795, 652),
    ("prateleira", 1088, 488, 1152, 548),
]
# ACESSOS NA DIVISÓRIA: sala <-> quarto e sala <-> banheiro.
# Cada passagem abre um vão de 15 px na parede vertical x=720..735.
# Ajuste y0/y1 para reposicionar ou aumentar/reduzir as aberturas.
PASSAGENS = [
    (727, 235, 325),  # porta de acesso ao quarto
    (727, 505, 595),  # porta de acesso ao banheiro
]
PASSAGENS_ATIVAS = True
PORTAS = PASSAGENS if PASSAGENS_ATIVAS else []

COMODOS = {   # nome -> (x, y) em pixels da planta
    "Cozinha": (520, 238), "Jantar": (385, 352), "Sala": (535, 470),
    "Quarto": (880, 165), "Banheiro": (870, 570), "Entrada": (650, 620),
}
START_PX = (520, 340)
TOUR = ["Cozinha", "Jantar", "Sala", "Banheiro", "Quarto", "Entrada"]


# ============================================================================
# 3. UTILIDADES DE COORDENADAS (mundo em metros, y para CIMA)  <->  planta (px)
# ============================================================================
def px2m(px, py):
    return px / PX_PER_M, (IMG_H - py) / PX_PER_M


def m2px(x, y):
    return x * PX_PER_M, IMG_H - y * PX_PER_M


def m2cell(x, y):
    px, py = m2px(x, y)
    return int(px // CELL), int(py // CELL)


def cell2m(cx, cy):
    return px2m((cx + 0.5) * CELL, (cy + 0.5) * CELL)


# ============================================================================
# 4. FUNÇÕES DA MATÉRIA (Aulas 2-5)  - usadas de verdade pelos nós abaixo
# ============================================================================
def normalizar_angulo(a):
    """Menor diferença angular em [-pi, pi]."""
    return (a + math.pi) % (2 * math.pi) - math.pi


def converter_cmd_vel(v, omega, L=WHEELBASE, max_wheel_speed=MAX_WHEEL):
    """AC-2 Ex.1: (v, omega) -> (v_e, v_d) com saturação proporcional."""
    v_d = v + omega * L / 2.0
    v_e = v - omega * L / 2.0
    m = max(abs(v_e), abs(v_d))
    if m > max_wheel_speed:                     # satura mantendo a curvatura
        k = max_wheel_speed / m
        v_e, v_d = v_e * k, v_d * k
    return v_e, v_d


def processar_scan(leituras):
    """AC-2 Ex.2: filtra o LiDAR (descarta 0.0 e inf) e devolve o menor valor
    válido de 3 setores. Índice k = graus anti-horário a partir da frente."""
    l = np.asarray(leituras, dtype=float)
    ok = np.isfinite(l) & (l >= 0.1) & (l <= 5.0)

    def menor(idx):
        v = l[idx][ok[idx]]
        return float(v.min()) if v.size else LIDAR_RANGE

    frente = np.r_[345:360, 0:16]
    esq = np.arange(45, 136)
    dire = np.arange(225, 316)
    return {"frente": menor(frente), "esquerda": menor(esq), "direita": menor(dire)}


def controle_reativo(d, v_livre=0.5, d_critica=D_CRITICA):
    """AC-2 Ex.3: Braitenberg + trava de segurança. Retorna (v, omega).
    omega > 0 = girar para a ESQUERDA (convenção ROS)."""
    if d["frente"] < d_critica:
        return 0.0, (1.0 if d["esquerda"] >= d["direita"] else -1.0)
    return v_livre, float(np.clip(0.8 * (d["esquerda"] - d["direita"]), -1, 1))


def calcular_orientacao_alvo(x, y, theta, x_alvo, y_alvo, Kp=KP_ANG):
    """AC-2 Ex.4: controle P de orientação (Go-to-Goal). Retorna (omega, e_theta)."""
    theta_alvo = math.atan2(y_alvo - y, x_alvo - x)
    e = normalizar_angulo(theta_alvo - theta)
    return Kp * e, e


# ============================================================================
# 5. MUNDO (obstáculos), ROBÔ (física) E LIDAR
# ============================================================================
def pool(a):
    """Reduz um array (MAPH, MAPW) para a grade de células (qualquer px ocupado)."""
    return a.reshape(GR, CELL, GC, CELL).any(axis=(1, 3))


def shift(g, dy, dx):
    out = np.zeros_like(g)
    H, W = g.shape
    ys, yd = (slice(dy, H), slice(0, H - dy)) if dy >= 0 else (slice(0, H + dy), slice(-dy, H))
    xs, xd = (slice(dx, W), slice(0, W - dx)) if dx >= 0 else (slice(0, W + dx), slice(-dx, W))
    out[ys, xs] = g[yd, xd]
    return out


def dilate(g, r):
    """Dilatação circular (raio r em células) - inflação de obstáculos."""
    out = g.copy()
    R = int(math.ceil(r))
    for dy in range(-R, R + 1):
        for dx in range(-R, R + 1):
            if (dx or dy) and dx * dx + dy * dy <= r * r + 1e-9:
                out |= shift(g, dy, dx)
    return out


def raster_poligono(pts):
    """Rasteriza um polígono (px da planta) numa máscara booleana (MAPH, MAPW)."""
    surf = pygame.Surface((MAPW, MAPH), depth=8)
    surf.fill(0)
    pygame.draw.polygon(surf, 1, pts)
    return pygame.surfarray.array2d(surf).T.astype(bool)


def alcancaveis(livre, inicio):
    """Flood-fill 4-conexo: células livres alcançáveis a partir de 'inicio'."""
    H, W = livre.shape
    ok = np.zeros_like(livre)
    s = nearest_free(~livre, inicio, 30)
    if s is None:
        return ok
    ok[s[1], s[0]] = True
    q = deque([s])
    while q:
        x, y = q.popleft()
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nx, ny = x + dx, y + dy
            if 0 <= nx < W and 0 <= ny < H and livre[ny, nx] and not ok[ny, nx]:
                ok[ny, nx] = True
                q.append((nx, ny))
    return ok


class World:
    def __init__(self):
        self.static = np.zeros((MAPH, MAPW), bool)
        for _, x0, y0, x1, y1 in PAREDES + MOVEIS:
            self.static[y0:y1, x0:x1] = True
        for _, pts in PAREDES_POLI:
            self.static |= raster_poligono(pts)
        for x, y0, y1 in PORTAS:                       # aberturas opcionais (PASSAGENS_ATIVAS)
            self.static[y0:y1, 720:735] = False
        self.disc = None                       # (x, y, r) em metros: pessoa/intruso em movimento
        self.boxes = []                       # obstáculos surpresa (pygame.Rect)
        self.occ = self.static.copy()
        self.static_cells = pool(self.static)
        ys, xs = np.mgrid[0:GR, 0:GC]
        inside = (xs * CELL >= 57) & (xs * CELL < 1218) & (ys * CELL >= 33) & (ys * CELL < 690)
        self.free_cells = inside & ~self.static_cells      # área navegável (p/ cobertura)
        sc = (int(START_PX[0] // CELL), int(START_PX[1] // CELL))
        # cobertura só conta o que o robô realmente alcança (cômodos isolados por parede ficam de fora)
        alc = alcancaveis(~dilate(self.static_cells, INFLATE_CELLS), sc)
        self.free_cells = self.free_cells & dilate(alc, INFLATE_CELLS + 1)

    def rebuild(self):
        self.occ = self.static.copy()
        for r in self.boxes:
            self.occ[r.top:r.bottom, r.left:r.right] = True

    def toggle_box(self, px, py, robot_xy, size=38):
        for r in self.boxes:
            if r.collidepoint(px, py):
                self.boxes.remove(r)
                self.rebuild()
                return "removida"
        r = pygame.Rect(0, 0, size, size)
        r.center = (int(px), int(py))
        if self.static[r.top:r.bottom, r.left:r.right].any():
            return "ocupado"
        rx, ry = m2px(*robot_xy)
        if math.hypot(rx - px, ry - py) < 45:
            return "perto"
        self.boxes.append(r)
        self.rebuild()
        return "criada"

    def is_occ(self, px, py):
        if px < 0 or py < 0 or px >= MAPW or py >= MAPH:
            return True
        return bool(self.occ[int(py), int(px)])

    def overlap(self, x, y, r):
        """Quantos pontos do contorno do robô estão dentro de obstáculos."""
        px, py = m2px(x, y)
        rp = r * PX_PER_M
        n = int(self.is_occ(px, py))
        for k in range(16):
            a = k * math.pi / 8
            n += int(self.is_occ(px + rp * math.cos(a), py - rp * math.sin(a)))
        return n

    def circle_free(self, x, y, r):
        px, py = m2px(x, y)
        rp = r * PX_PER_M
        if self.disc and math.hypot(x - self.disc[0], y - self.disc[1]) < r + self.disc[2]:
            return False                                     # não atravessa a pessoa/intruso
        if self.is_occ(px, py):
            return False
        for k in range(16):
            a = k * math.pi / 8
            if self.is_occ(px + rp * math.cos(a), py - rp * math.sin(a)):
                return False
        return True


def lidar_scan(world, x, y, th, noise=True):
    """Aula 03: 360 raios por raycasting (amostragem ao longo do raio, vetorizado).
    Retorna array de 360 distâncias: inf = sem retorno, 0.0 = falha do sensor."""
    ang = th + np.radians(np.arange(360))
    steps = np.arange(LIDAR_MIN, LIDAR_RANGE, 0.03)
    xs = x + np.cos(ang)[:, None] * steps[None, :]
    ys = y + np.sin(ang)[:, None] * steps[None, :]
    px = (xs * PX_PER_M).astype(int)
    py = (IMG_H - ys * PX_PER_M).astype(int)
    inb = (px >= 0) & (px < MAPW) & (py >= 0) & (py < MAPH)
    hit = np.ones(px.shape, bool)
    hit[inb] = world.occ[py[inb], px[inb]]
    anyhit = hit.any(1)
    first = hit.argmax(1)
    r = np.where(anyhit, steps[first], np.inf)
    dsc = getattr(world, "disc", None)
    if dsc:                                   # interseção raio x círculo (pessoa/intruso)
        ux, uy = np.cos(ang), np.sin(ang)
        ox, oy = x - dsc[0], y - dsc[1]
        b = ux * ox + uy * oy
        delta = b * b - (ox * ox + oy * oy - dsc[2] ** 2)
        t = -b - np.sqrt(np.where(delta >= 0, delta, 0.0))
        ok = (delta >= 0) & (t >= LIDAR_MIN) & (t <= LIDAR_RANGE)
        r = np.where(ok, np.minimum(r, t), r)
    if noise:
        fin = np.isfinite(r)
        r[fin] += np.random.normal(0, LIDAR_NOISE, fin.sum())
        r[np.random.rand(360) < LIDAR_DROP] = 0.0
    return r


class Robot:
    """Física 'real' do robô (Aula 02): rodas -> v, omega -> integração da pose."""

    def __init__(self, x, y, th):
        self.x, self.y, self.th = x, y, th
        self.tl = self.tr = 0.0        # alvo das rodas (comando do motor)
        self.vl = self.vr = 0.0        # velocidade real das rodas (encoders leem isto)
        self.v = self.w = 0.0
        self.collisions = 0
        self.dist = 0.0
        self._colliding = False

    def step(self, dt, world, noise=True):
        a = MAX_ACC * dt
        self.vl += float(np.clip(self.tl - self.vl, -a, a))
        self.vr += float(np.clip(self.tr - self.vr, -a, a))
        sl, sr = SLIP if noise else (1.0, 1.0)
        vl, vr = self.vl * sl, self.vr * sr
        v, w = (vr + vl) / 2.0, (vr - vl) / WHEELBASE
        th2 = self.th + w * dt
        x2 = self.x + v * math.cos(self.th + w * dt / 2) * dt
        y2 = self.y + v * math.sin(self.th + w * dt / 2) * dt
        free_new = world.circle_free(x2, y2, ROBOT_R)
        escaping = (not free_new and self._colliding and
                    world.overlap(x2, y2, ROBOT_R) <= world.overlap(self.x, self.y, ROBOT_R))
        if free_new or escaping:
            self.dist += math.hypot(x2 - self.x, y2 - self.y)
            self.x, self.y, self.th = x2, y2, normalizar_angulo(th2)
            self.v, self.w = v, w
            self._colliding = not free_new
        else:                                    # bateu: não deixa atravessar
            self.th = normalizar_angulo(th2)
            self.v, self.w = 0.0, w
            if not self._colliding:
                self.collisions += 1
            self._colliding = True


# ============================================================================
# 6. PLANEJAMENTO (IA): A* + suavização por linha de visada
# ============================================================================
def nearest_free(blocked, cell, maxr=40):
    H, W = blocked.shape
    cx, cy = cell
    cx, cy = min(max(cx, 0), W - 1), min(max(cy, 0), H - 1)
    if not blocked[cy, cx]:
        return (cx, cy)
    seen = {(cx, cy)}
    q = deque([(cx, cy, 0)])
    while q:
        x, y, d = q.popleft()
        if d > maxr:
            break
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nx, ny = x + dx, y + dy
            if 0 <= nx < W and 0 <= ny < H and (nx, ny) not in seen:
                if not blocked[ny, nx]:
                    return (nx, ny)
                seen.add((nx, ny))
                q.append((nx, ny, d + 1))
    return None


def astar(blocked, near, start, goal):
    """A* em grade 8-conectada. 'near' = células perto de parede (custo extra:
    o caminho tende a passar pelo meio dos cômodos e portas)."""
    H, W = blocked.shape
    blk = blocked.ravel().tolist()
    pen = (near.ravel() * 2.5).tolist()
    s, g = start[1] * W + start[0], goal[1] * W + goal[0]
    gx, gy = goal
    INF = 1e18
    gs = {s: 0.0}
    parent = {s: -1}
    openq = [(0.0, s)]
    closed = set()
    D = 1.4142
    nbrs = ((1, 0, 1), (-1, 0, 1), (0, 1, 1), (0, -1, 1),
            (1, 1, D), (1, -1, D), (-1, 1, D), (-1, -1, D))
    while openq:
        _, cur = heapq.heappop(openq)
        if cur in closed:
            continue
        if cur == g:
            path = []
            while cur != -1:
                path.append((cur % W, cur // W))
                cur = parent[cur]
            return path[::-1]
        closed.add(cur)
        cx, cy = cur % W, cur // W
        for dx, dy, c in nbrs:
            nx, ny = cx + dx, cy + dy
            if nx < 0 or ny < 0 or nx >= W or ny >= H:
                continue
            ni = ny * W + nx
            if blk[ni] or ni in closed:
                continue
            if dx and dy and (blk[cy * W + nx] or blk[ny * W + cx]):
                continue                           # não corta quina
            ng = gs[cur] + c + pen[ni]
            if ng < gs.get(ni, INF):
                gs[ni] = ng
                parent[ni] = cur
                ddx, ddy = abs(nx - gx), abs(ny - gy)
                h = (ddx + ddy) + (D - 2) * min(ddx, ddy)
                heapq.heappush(openq, (ng + 1.1 * h, ni))
    return None


def line_free(blocked, a, b):
    n = int(max(abs(b[0] - a[0]), abs(b[1] - a[1])) * 2) + 1
    for i in range(n + 1):
        t = i / n
        x = int(round(a[0] + (b[0] - a[0]) * t))
        y = int(round(a[1] + (b[1] - a[1]) * t))
        if blocked[y, x]:
            return False
    return True


def smooth_path(cells, blocked):
    out, i = [cells[0]], 0
    while i < len(cells) - 1:
        j = len(cells) - 1
        while j > i + 1 and not line_free(blocked, cells[i], cells[j]):
            j -= 1
        out.append(cells[j])
        i = j
    return out


# ============================================================================
# 7. ARQUITETURA ESTILO ROS 2: barramento de tópicos + nós
# ============================================================================
class Bus:
    """Mini publish/subscribe (conceito de tópicos do ROS 2)."""

    def __init__(self):
        self.subs, self.last, self.cnt, self.rate, self.stamp = {}, {}, {}, {}, {}
        self.t0 = 0.0
        self.now = 0.0

    def sub(self, topic, cb):
        self.subs.setdefault(topic, []).append(cb)

    def pub(self, topic, msg):
        self.last[topic] = msg
        self.cnt[topic] = self.cnt.get(topic, 0) + 1
        self.stamp[topic] = self.now
        for cb in self.subs.get(topic, []):
            cb(msg)

    def tick(self, now):
        self.now = now
        if now - self.t0 >= 1.0:
            dt = now - self.t0
            self.rate = {k: v / dt for k, v in self.cnt.items()}
            self.cnt = {k: 0 for k in self.cnt}
            self.t0 = now


class Params:
    def __init__(self):
        self.use_odom = False        # controlar/mapear pela odometria (com deriva)?
        self.noise = True
        self.mode = "OCIOSO"         # OCIOSO | IR | PATRULHA | EXPLORAR | MANUAL
        self.manual = (0.0, 0.0)
        self.night = False           # modo noturno (vigilância reforçada)


class Node:
    period = 0.1

    def __init__(self, sim):
        self.sim, self.bus, self.acc = sim, sim.bus, 0.0

    def tick(self):
        pass


class MotorDriverNode(Node):
    """/cmd_vel -> rodas (usa converter_cmd_vel da AC-2)."""
    period = 1 / 60

    def __init__(self, sim):
        super().__init__(sim)
        self.cmd, self.age = (0.0, 0.0), 0.0
        self.bus.sub("/cmd_vel", self.on_cmd)

    def on_cmd(self, m):
        self.cmd, self.age = m, 0.0

    def tick(self):
        self.age += self.period
        v, w = self.cmd if self.age < 0.5 else (0.0, 0.0)     # watchdog
        ve, vd = converter_cmd_vel(v, w)
        self.sim.robot.tl, self.sim.robot.tr = ve, vd
        self.bus.pub("/wheel_cmd", (ve, vd))


class EncoderOdomNode(Node):
    """Odometria por integração das rodas (Aula 02) -> /odom (acumula deriva)."""
    period = 1 / 30

    def __init__(self, sim):
        super().__init__(sim)
        self.x, self.y, self.th = sim.robot.x, sim.robot.y, sim.robot.th

    def tick(self):
        r = self.sim.robot
        n = 0.003 if self.sim.p.noise else 0.0
        vl, vr = r.vl + random.gauss(0, n), r.vr + random.gauss(0, n)
        v, w = (vr + vl) / 2.0, (vr - vl) / WHEELBASE
        self.th = normalizar_angulo(self.th + w * self.period)
        self.x += v * math.cos(self.th) * self.period
        self.y += v * math.sin(self.th) * self.period
        self.bus.pub("/odom", (self.x, self.y, self.th, v, w))

    def relocalize(self):
        r = self.sim.robot
        self.x, self.y, self.th = r.x, r.y, r.th


class GroundTruthNode(Node):
    period = 1 / 30

    def tick(self):
        r = self.sim.robot
        self.bus.pub("/ground_truth", (r.x, r.y, r.th))


class LidarNode(Node):
    period = 0.1

    def tick(self):
        r = self.sim.robot
        self.bus.pub("/scan", lidar_scan(self.sim.world, r.x, r.y, r.th, self.sim.p.noise))


class PerceptionNode(Node):
    """/scan -> /scan_sectors (filtro + setores, AC-2 Ex.2)."""

    def __init__(self, sim):
        super().__init__(sim)
        self.bus.sub("/scan", lambda s: self.bus.pub("/scan_sectors", processar_scan(s)))


class MapperNode(Node):
    """Mapeamento por grade de ocupação (log-odds) a partir do /scan."""
    period = 0.5

    def __init__(self, sim):
        super().__init__(sim)
        self.L = np.zeros((GR, GC), np.float32)
        self.seen = np.zeros((GR, GC), bool)
        self.coverage = 0.0
        self.bus.sub("/scan", self.on_scan)

    def on_scan(self, scan):
        x, y, th = self.sim.est_pose()
        n = len(scan)
        ang = th + np.radians(np.arange(n))
        rr = np.where(np.isfinite(scan), scan, LIDAR_RANGE)
        valid = (scan >= LIDAR_MIN) & ~self.sim.security.dyn   # descarta 0.0 (falha) e pessoas em movimento
        hit = np.isfinite(scan) & valid
        d = np.arange(0.06, LIDAR_RANGE, CELL / PX_PER_M * 0.9)
        m = (d[None, :] < (rr[:, None] - 0.05)) & valid[:, None]
        xs = x + np.cos(ang)[:, None] * d[None, :]
        ys = y + np.sin(ang)[:, None] * d[None, :]
        cx = (xs * PX_PER_M // CELL).astype(int)
        cy = ((IMG_H - ys * PX_PER_M) // CELL).astype(int)
        ok = m & (cx >= 0) & (cx < GC) & (cy >= 0) & (cy < GR)
        np.add.at(self.L, (cy[ok], cx[ok]), -0.45)
        self.seen[cy[ok], cx[ok]] = True
        hx = x + np.cos(ang[hit]) * rr[hit]
        hy = y + np.sin(ang[hit]) * rr[hit]
        hcx = (hx * PX_PER_M // CELL).astype(int)
        hcy = ((IMG_H - hy * PX_PER_M) // CELL).astype(int)
        k = (hcx >= 0) & (hcx < GC) & (hcy >= 0) & (hcy < GR)
        np.add.at(self.L, (hcy[k], hcx[k]), 1.0)
        self.seen[hcy[k], hcx[k]] = True
        np.clip(self.L, -3.0, 3.0, out=self.L)

    def occupied(self):
        return self.L > 0.8

    def free(self):
        return self.seen & (self.L < -0.4)

    def tick(self):
        w = self.sim.world
        tot = w.free_cells.sum()
        self.coverage = float((self.seen & w.free_cells).sum()) / max(1, tot)
        self.bus.pub("/map", {"coverage": self.coverage})

    def frontier(self, start_cell, blocked, blacklist):
        """Fronteira = célula livre conhecida vizinha de região desconhecida.
        BFS a partir do robô -> fronteira mais próxima e alcançável."""
        free = self.free() & ~blocked
        unk = ~self.seen
        near_unk = np.zeros_like(unk)
        for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            near_unk |= shift(unk, dy, dx)
        front = free & near_unk
        s = nearest_free(~free, start_cell, 6)
        if s is None:
            return None
        seen = {s}
        q = deque([s])
        while q:
            x, y = q.popleft()
            if front[y, x] and all(math.hypot(x - bx, y - by) > 7 for bx, by in blacklist):
                return (x, y)
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
                nx, ny = x + dx, y + dy
                if 0 <= nx < GC and 0 <= ny < GR and (nx, ny) not in seen and free[ny, nx]:
                    seen.add((nx, ny))
                    q.append((nx, ny))
        return None


class PlannerNode(Node):
    """/goal -> A* -> /path. Replaneja sozinho se o caminho for bloqueado."""
    period = 0.5

    def __init__(self, sim):
        super().__init__(sim)
        w = sim.world
        self.static_blocked = dilate(w.static_cells, INFLATE_CELLS)
        self.static_near = dilate(w.static_cells, 1.5)
        self.goal = None
        self.waypoints = []                 # em células
        self.dense = []
        self.map_kind = "planta"
        self.new_obs = np.zeros((GR, GC), bool)
        self.bus.sub("/goal", self.on_goal)
        self.bus.sub("/goal_reached", lambda m: self.clear())
        self.bus.sub("/replan_request", lambda m: self.plan())

    def clear(self):
        self.goal, self.waypoints = None, []

    def blocked_grid(self):
        mp = self.sim.mapper
        if self.map_kind == "aprendido":
            return dilate(mp.occupied(), INFLATE_CELLS) | ~mp.seen
        self.new_obs = mp.occupied() & ~self.static_near
        if self.new_obs.any():
            return self.static_blocked | dilate(self.new_obs, INFLATE_CELLS)
        return self.static_blocked

    def on_goal(self, m):
        self.goal = m
        self.map_kind = m.get("map", "planta")
        self.plan()

    def plan(self):
        if not self.goal:
            return
        blocked = self.blocked_grid()
        near = dilate(blocked, 2.5) & ~blocked
        x, y, _ = self.sim.est_pose()
        s = nearest_free(blocked, m2cell(x, y))
        gx, gy = self.goal["goal"]
        g = nearest_free(blocked, m2cell(gx, gy))
        cells = astar(blocked, near, s, g) if (s and g) else None
        if not cells:
            self.waypoints, self.dense = [], []
            self.bus.pub("/path", {"pts": [], "name": self.goal.get("name", "")})
            return
        self.dense = cells
        self.waypoints = smooth_path(cells, blocked)
        pts = [cell2m(*c) for c in self.waypoints]
        length = sum(math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
                     for i in range(len(pts) - 1))
        self.bus.pub("/path", {"pts": pts, "name": self.goal.get("name", ""), "length": length})

    def tick(self):
        if self.goal and not self.waypoints:       # sem rota: tenta de novo (caixa pode ter saído)
            self.retry = getattr(self, "retry", 0) + 1
            if self.retry % 2 == 0:
                self.plan()
            return
        if not self.goal:
            return
        blocked = self.blocked_grid()
        for i in range(len(self.waypoints) - 1):
            if not line_free(blocked, self.waypoints[i], self.waypoints[i + 1]):
                self.sim.log("Planner: caminho bloqueado -> replanejando (A*)")
                self.plan()
                return


class BrainNode(Node):
    """FSM + controlador (AC-2 Ex.3/4/5) -> /cmd_vel."""
    period = 1 / 30

    def __init__(self, sim):
        super().__init__(sim)
        self.estado, self.path, self.idx = "PARADO", [], 1
        self.goal_name, self.cool, self.dir = "", 0.0, 1
        self.sec = {"frente": 5.0, "esquerda": 5.0, "direita": 5.0}
        self.hist = deque(maxlen=90)
        self.bus.sub("/scan_sectors", lambda s: setattr(self, "sec", s))
        self.bus.sub("/path", self.on_path)

    def on_path(self, m):
        self.goal_name = m.get("name", "")
        if not m["pts"]:
            if self.estado != "SEM_ROTA":
                self.sim.log(f"Sem rota para {self.goal_name or 'o destino'}! (vou tentar de novo)")
            self.path, self.estado = [], "SEM_ROTA"
            return
        self.path, self.idx = m["pts"], 1
        if self.estado != "DESVIAR_OBSTACULO":
            self.estado = "IR_PARA_ALVO"
        self.hist.clear()

    def start_goal(self):
        self.estado = "IR_PARA_ALVO"

    def step_fsm(self, x, y, th):
        """Retorna (v, omega) e atualiza o estado (AC-2 Ex.5)."""
        sec, v, w = self.sec, 0.0, 0.0
        if self.cool > 0:
            self.cool -= self.period
        if self.estado == "DESVIAR_OBSTACULO":
            v, w = 0.0, 1.0 * self.dir               # freia e gira p/ lado livre
            if sec["frente"] > D_SAIDA:
                self.estado, self.cool = "IR_PARA_ALVO", 1.2
                self.sim.log("Frente livre -> replanejando a rota")
                self.bus.pub("/replan_request", True)
        elif self.estado == "IR_PARA_ALVO" and self.path:
            gx, gy = self.path[-1]
            while self.idx < len(self.path) - 1 and \
                    math.hypot(self.path[self.idx][0] - x, self.path[self.idx][1] - y) < TOL_WP:
                self.idx += 1
            if math.hypot(gx - x, gy - y) < TOL_FINAL:
                self.estado = "OBJETIVO_ALCANÇADO"
                self.sim.log(f"Chegamos: {self.goal_name}")
                self.bus.pub("/goal_reached", self.goal_name)
                return 0.0, 0.0
            if self.cool <= 0 and sec["frente"] < D_CRITICA:
                self.estado = "DESVIAR_OBSTACULO"
                self.dir = 1 if sec["esquerda"] >= sec["direita"] else -1
                self.sim.log(f"Obstáculo a {sec['frente']:.2f} m! Freio + giro")
                return 0.0, 1.0 * self.dir
            # --- seguimento do segmento A->B: mira num ponto "cenoura" à frente na reta,
            #     o que corrige o desvio lateral e evita cortar quinas (AULA 02: Go-to-Goal P)
            ax, ay = self.path[self.idx - 1]
            bx, by = self.path[self.idx]
            seg = math.hypot(bx - ax, by - ay)
            if seg > 1e-6:
                ux, uy = (bx - ax) / seg, (by - ay) / seg
                t = min(max((x - ax) * ux + (y - ay) * uy, 0.0), seg)
                tc = min(seg, t + LOOKAHEAD)
                tx, ty = ax + ux * tc, ay + uy * tc
            else:
                tx, ty = bx, by
            w, e = calcular_orientacao_alvo(x, y, th, tx, ty)
            w = float(np.clip(w, -W_MAX, W_MAX))
            dB = math.hypot(bx - x, by - y)
            v = float(np.clip(KP_DIST * dB, 0.12, V_MAX)) * max(0.0, math.cos(e)) ** 3
            if self.idx < len(self.path) - 1 and dB < 0.45 and seg > 1e-6:     # curva à frente
                cx_, cy_ = self.path[self.idx + 1]
                s2 = math.hypot(cx_ - bx, cy_ - by)
                if s2 > 1e-6:
                    ct = ((bx - ax) * (cx_ - bx) + (by - ay) * (cy_ - by)) / (seg * s2)
                    turn = math.acos(max(-1.0, min(1.0, ct)))
                    v *= max(0.3, 1.0 - turn / math.pi)
            v *= float(np.clip((sec["frente"] - 0.22) / 0.6, 0.25, 1.0))   # reduz perto de coisas
            # detector de "preso"
            self.hist.append((x, y))
            if len(self.hist) == self.hist.maxlen and \
                    math.hypot(x - self.hist[0][0], y - self.hist[0][1]) < 0.04:
                self.hist.clear()
                self.sim.log("Robô parado há 3 s -> replanejando")
                self.bus.pub("/replan_request", True)
        if self.sim.p.night:
            v = min(v, V_NOITE)                       # ronda noturna: mais devagar e atento
        return v, w

    def tick(self):
        p = self.sim.p
        if p.mode == "MANUAL":
            self.estado = "MANUAL"
            self.bus.pub("/cmd_vel", p.manual)
        else:
            if self.estado == "MANUAL":
                self.estado = "PARADO"
            x, y, th = self.sim.est_pose()
            self.bus.pub("/cmd_vel", self.step_fsm(x, y, th))
        self.bus.pub("/vigilar/status", self.estado)


class MissionNode(Node):
    """Quem decide os destinos: clique, patrulha ou exploração autônoma."""
    period = 0.4

    def __init__(self, sim):
        super().__init__(sim)
        self.tour_i, self.wait, self.blacklist, self.cur_frontier = 0, 0.0, [], None
        self.done_explore = False

    def send_goal(self, name, pos_m, kind="planta"):
        self.bus.pub("/goal", {"goal": pos_m, "name": name, "map": kind})
        self.sim.brain.start_goal()
        self.sim.log(f"Missão: ir para {name}")

    def goto_px(self, name, px, py):
        self.send_goal(name, px2m(px, py))

    def tick(self):
        p, br = self.sim.p, self.sim.brain
        idle = br.estado in ("OBJETIVO_ALCANÇADO", "SEM_ROTA", "PARADO")
        if p.mode == "PATRULHA" and idle:
            self.wait += self.period
            if self.wait >= 1.5:
                self.wait = 0.0
                tour = self.sim.rooms_ok or TOUR
                n = tour[self.tour_i % len(tour)]
                self.tour_i += 1
                self.goto_px(n, *COMODOS[n])
        elif p.mode == "EXPLORAR" and idle:
            if br.estado == "SEM_ROTA" and self.cur_frontier:
                self.blacklist.append(self.cur_frontier)
            elif br.estado == "OBJETIVO_ALCANÇADO" and self.cur_frontier:
                self.blacklist.append(self.cur_frontier)
            self.cur_frontier = None
            mp = self.sim.mapper
            x, y, _ = self.sim.est_pose()
            blocked = dilate(mp.occupied(), INFLATE_CELLS) | ~mp.seen
            f = mp.frontier(m2cell(x, y), blocked, self.blacklist)
            if f is None:
                p.mode = "OCIOSO"
                br.estado = "PARADO"
                self.done_explore = True
                self.sim.log(f"Exploração concluída! Cobertura {mp.coverage * 100:.0f}%")
            else:
                self.cur_frontier = f
                self.send_goal("fronteira", cell2m(*f), "aprendido")


def comodo_em(x, y):
    """Nome do cômodo para uma posição (m) - usado nas mensagens de alerta."""
    px, py = m2px(x, y)
    if px >= 735:
        return "Quarto" if py < 450 else "Banheiro"
    return min(("Cozinha", "Jantar", "Sala", "Entrada"),
               key=lambda n: math.hypot(px - COMODOS[n][0], py - COMODOS[n][1]))


def enviar_alerta(a):
    """'Envio' do alerta: grava em arquivo (JSON por linha) e, se VIGILAR_WEBHOOK
    estiver definida, faz POST JSON nela. Roda em thread para não travar a simulação."""
    try:
        with open(ALERT_LOG, "a", encoding="utf-8") as f:
            f.write(json.dumps(a, ensure_ascii=False) + "\n")
    except OSError:
        pass
    if ALERT_WEBHOOK:
        try:
            req = urllib.request.Request(ALERT_WEBHOOK, json.dumps(a).encode(),
                                         {"Content-Type": "application/json"})
            urllib.request.urlopen(req, timeout=3).close()
        except Exception:
            pass


class SecurityNode(Node):
    """Vigilância: compara o /scan MEDIDO com o /scan ESPERADO da planta (só paredes e
    móveis). Raios bem mais curtos que o esperado = algo novo. Agrupa em alvos, acompanha
    cada alvo e mede o deslocamento: parado = 'objeto novo'; em movimento = pessoa.
    À noite, movimento => ALERTA de movimentação suspeita (/alerta)."""

    def __init__(self, sim):
        super().__init__(sim)
        self.estatico = types.SimpleNamespace(occ=sim.world.static, disc=None)
        self.dyn = np.zeros(360, bool)           # raios que pertencem a alvos em movimento
        self.tracks, self.next_id = [], 1
        self.alertas = deque(maxlen=40)
        self.n_alertas = 0
        self._aviso_odom = False
        self.bus.sub("/scan", self.on_scan)

    def _clusters(self, m):
        idx = np.where(m)[0]
        if idx.size == 0:
            return []
        cl, cur = [], [int(idx[0])]
        for i in idx[1:]:
            if i - cur[-1] <= 3:
                cur.append(int(i))
            else:
                cl.append(cur)
                cur = [int(i)]
        cl.append(cur)
        if len(cl) > 1 and cl[0][0] <= 3 and cl[-1][-1] >= 356:     # fecha o círculo 359 -> 0
            cl[0] = cl.pop() + cl[0]
        return [c for c in cl if len(c) >= 2]

    def on_scan(self, scan):
        sim, t = self.sim, self.bus.now
        self.dyn[:] = False
        if sim.p.use_odom and sim.odom_error()[0] > 0.10:
            if not self._aviso_odom:
                sim.log("Vigilância pausada: odometria derivou (use Z para relocalizar)")
                self._aviso_odom = True
            return
        self._aviso_odom = False
        x, y, th = sim.est_pose()
        esp = lidar_scan(self.estatico, x, y, th, noise=False)
        esp = np.where(np.isfinite(esp), esp, LIDAR_RANGE)
        m = np.isfinite(scan) & (scan >= LIDAR_MIN) & (scan < esp - ANOM_MARGEM)
        vistos = set()
        for cl in self._clusters(m):
            ang = th + np.radians(cl)
            cx = float(np.mean(x + np.cos(ang) * scan[cl]))
            cy = float(np.mean(y + np.sin(ang) * scan[cl]))
            tr = min((q for q in self.tracks if q["id"] not in vistos),
                     key=lambda q: math.hypot(q["pos"][0] - cx, q["pos"][1] - cy), default=None)
            if tr is None or math.hypot(tr["pos"][0] - cx, tr["pos"][1] - cy) > 0.6:
                tr = {"id": self.next_id, "t0": t, "hist": deque(), "mov": False, "was_mov": False,
                      "info": False, "last_alert": -99.0, "pos": (cx, cy), "seen": t}
                self.next_id += 1
                self.tracks.append(tr)
            vistos.add(tr["id"])
            tr["pos"], tr["seen"] = (cx, cy), t
            tr["hist"].append((t, cx, cy))
            while tr["hist"] and t - tr["hist"][0][0] > MOVE_JANELA:
                tr["hist"].popleft()
            h = tr["hist"]
            tr["mov"] = (t - tr["t0"] >= MOVE_JANELA * 0.8 and t - h[0][0] >= MOVE_JANELA * 0.8
                         and math.hypot(cx - h[0][1], cy - h[0][2]) > MOVE_DESLOC)
            tr["was_mov"] |= tr["mov"]
            if tr["mov"]:
                self.dyn[cl] = True
                if sim.p.night and t - tr["last_alert"] >= ALERTA_COOLDOWN:
                    tr["last_alert"] = t
                    self.emitir("CRITICO", "MOVIMENTO_SUSPEITO", (cx, cy),
                                "Movimentação suspeita detectada durante o modo noturno")
            elif not tr["was_mov"] and not tr["info"] and t - tr["t0"] >= 2.5:
                tr["info"] = True
                self.emitir("INFO", "OBJETO_NOVO", (cx, cy), "Objeto novo e parado fora da planta")
        self.tracks = [q for q in self.tracks if t - q["seen"] < 1.0]

    def emitir(self, nivel, tipo, pos, msg):
        sim = self.sim
        a = {"tempo_sim": round(self.bus.now, 1), "hora": time.strftime("%Y-%m-%d %H:%M:%S"),
             "nivel": nivel, "tipo": tipo, "x": round(pos[0], 2), "y": round(pos[1], 2),
             "comodo": comodo_em(*pos), "modo": "NOITE" if sim.p.night else "DIA", "msg": msg}
        self.alertas.append(a)
        if nivel != "INFO":
            self.n_alertas += 1
        sim.log(f"ALERTA [{nivel}] {tipo} em {a['comodo']} ({a['x']:.1f}, {a['y']:.1f}) -> enviado")
        self.bus.pub("/alerta", a)
        threading.Thread(target=enviar_alerta, args=(a,), daemon=True).start()


class IntruderNode(Node):
    """Pessoa simulada para TESTAR a vigilância (tecla I): entra pela Entrada e caminha
    entre os cômodos alcançáveis. Cede passagem se o robô estiver muito perto."""
    period = 1 / 30

    def __init__(self, sim):
        super().__init__(sim)
        self.on, self.x, self.y = False, 0.0, 0.0
        self.path, self.i, self.wait, self.room = [], 0, 0.0, None

    def toggle(self):
        w = self.sim.world
        if self.on:
            self.on, w.disc = False, None
            return False
        ok = self.sim.rooms_ok
        if not ok:
            return None
        r = self.sim.robot
        dist = lambda n: math.hypot(px2m(*COMODOS[n])[0] - r.x, px2m(*COMODOS[n])[1] - r.y)
        # entra pela Entrada, a menos que o robô esteja ali perto: aí nasce no cômodo mais distante
        self.room = "Entrada" if ("Entrada" in ok and dist("Entrada") > 1.5) else max(ok, key=dist)
        self.x, self.y = px2m(*COMODOS[self.room])
        self.path, self.wait, self.on = [], 0.5, True
        w.disc = (self.x, self.y, INTRUSO_R)
        return True

    def _destino(self):
        blk = self.sim.planner.static_blocked
        opts = [n for n in self.sim.rooms_ok if n != self.room]
        if not opts:
            return
        n = random.choice(opts)
        s = nearest_free(blk, m2cell(self.x, self.y))
        g = nearest_free(blk, (int(COMODOS[n][0] // CELL), int(COMODOS[n][1] // CELL)))
        cells = astar(blk, np.zeros_like(blk), s, g) if (s and g) else None
        if cells:
            self.path, self.i, self.room = [cell2m(*c) for c in cells], 0, n

    def tick(self):
        if not self.on:
            return
        r, dt = self.sim.robot, self.period
        if math.hypot(r.x - self.x, r.y - self.y) < ROBOT_R + INTRUSO_R + 0.25:
            return                                           # cede passagem ao robô
        if self.wait > 0:
            self.wait -= dt
            return
        if not self.path:
            self._destino()
            if not self.path:
                self.wait = 2.0
                return
        tx, ty = self.path[self.i]
        d, passo = math.hypot(tx - self.x, ty - self.y), INTRUSO_V * dt
        if d <= passo:
            self.x, self.y, self.i = tx, ty, self.i + 1
            if self.i >= len(self.path):
                self.path, self.wait = [], random.uniform(1.5, 3.5)
        else:
            self.x += (tx - self.x) / d * passo
            self.y += (ty - self.y) / d * passo
        self.sim.world.disc = (self.x, self.y, INTRUSO_R)
        self.bus.pub("/intruso_gt", (self.x, self.y))


# ============================================================================
# 8. SIMULAÇÃO (agrupa mundo, robô e nós)
# ============================================================================
class Sim:
    DT = 1 / 60

    def __init__(self):
        self.world = World()
        self.p = Params()
        self.logs = deque(maxlen=6)
        self.reset()

    def log(self, msg):
        self.logs.append(f"[{self.t:6.1f}s] {msg}")

    def reset(self, keep_boxes=False):
        if not keep_boxes:
            self.world.boxes = []
            self.world.rebuild()
        self.t = 0.0
        self.bus = Bus()
        x, y = px2m(*START_PX)
        self.robot = Robot(x, y, 0.0)
        self.p.mode = "OCIOSO"
        self.trail, self.odom_trail = [], []
        self.motor = MotorDriverNode(self)
        self.odom = EncoderOdomNode(self)
        self.gt = GroundTruthNode(self)
        self.lidar = LidarNode(self)
        self.perc = PerceptionNode(self)
        self.security = SecurityNode(self)          # antes do mapper: filtra raios móveis
        self.mapper = MapperNode(self)
        self.planner = PlannerNode(self)
        self.brain = BrainNode(self)
        self.mission = MissionNode(self)
        self.intruder = IntruderNode(self)
        self.world.disc = None
        self.p.night = False
        self.nodes = [self.odom, self.gt, self.lidar, self.mapper, self.planner,
                      self.brain, self.mission, self.intruder, self.motor]
        self._calc_rooms()
        self.logs.clear()
        self.log("vigiLAR pronto. P: ronda | T: modo noturno | I: intruso (teste) | 1-6: ir ao cômodo")
        if len(self.rooms_ok) < len(TOUR):
            self.log("Cômodos isolados por parede (fora da ronda): "
                     + ", ".join(n for n in TOUR if n not in self.rooms_ok))

    def _calc_rooms(self):
        """Quais cômodos da ronda o robô consegue alcançar (sem atravessar paredes)."""
        blk = self.planner.static_blocked
        s = nearest_free(blk, m2cell(self.robot.x, self.robot.y))
        alc = alcancaveis(~blk, s) if s else np.zeros_like(blk)
        self.rooms_ok = []
        for n in TOUR:
            g = nearest_free(blk, (int(COMODOS[n][0] // CELL), int(COMODOS[n][1] // CELL)), 8)
            if g and alc[g[1], g[0]]:
                self.rooms_ok.append(n)

    def est_pose(self):
        if self.p.use_odom:
            o = self.odom
            return o.x, o.y, o.th
        r = self.robot
        return r.x, r.y, r.th

    def odom_error(self):
        r, o = self.robot, self.odom
        return math.hypot(r.x - o.x, r.y - o.y), math.degrees(normalizar_angulo(r.th - o.th))

    def step(self, dt=None):
        dt = dt or self.DT
        self.t += dt
        self.bus.tick(self.t)
        self.robot.step(dt, self.world, self.p.noise)
        for n in self.nodes:
            n.acc += dt
            if n.acc >= n.period - 1e-9:
                n.acc -= n.period
                n.tick()
        if not self.trail or math.hypot(self.robot.x - self.trail[-1][0],
                                        self.robot.y - self.trail[-1][1]) > 0.05:
            self.trail.append((self.robot.x, self.robot.y))
            self.odom_trail.append((self.odom.x, self.odom.y))
            if len(self.trail) > 1500:
                self.trail.pop(0)
                self.odom_trail.pop(0)

    # comandos de alto nível usados pela interface
    def go_to_room(self, name):
        self.p.mode = "IR"
        self.mission.goto_px(name, *COMODOS[name])

    def go_to_px(self, px, py):
        self.p.mode = "IR"
        self.mission.goto_px("ponto clicado", px, py)

    def stop(self):
        self.p.mode = "OCIOSO"
        self.brain.estado = "PARADO"
        self.planner.clear()
        self.brain.path = []


# ============================================================================
# 9. INTERFACE (desenho + teclado/mouse)
# ============================================================================
COR_ESTADO = {
    "PARADO": (150, 150, 160), "IR_PARA_ALVO": (60, 200, 120),
    "DESVIAR_OBSTACULO": (255, 170, 40), "OBJETIVO_ALCANÇADO": (90, 160, 255),
    "SEM_ROTA": (240, 80, 80), "MANUAL": (200, 120, 255),
}
HELP = [
    "Clique: ir até lá | 1-6: Cozinha, Jantar, Sala, Quarto, Banheiro, Entrada | P: patrulha | E: explorar | TAB: manual (setas)",
    "T: modo NOTURNO (ronda + alertas) | I: intruso de teste | Botão dir.: caixa surpresa | V: vista | L: lidar | B: barreiras | O: odom.",
    "U: guiar pela odom. | Z: relocalizar | N: ruído | ESPAÇO: pausa | [ ]: velocidade | S: parar | R: reiniciar | ESC: sair",
]


class App:
    def __init__(self, scale=None):
        pygame.display.init()
        pygame.font.init()
        info = pygame.display.Info()
        if scale is None:
            sw, sh = info.current_w or 1600, info.current_h or 900
            scale = min(0.85, (sw - 40 - PANEL_W) / IMG_W, (sh - 110 - BOTTOM_H) / IMG_H)
        self.S = max(0.45, scale)
        self.iw, self.ih = int(IMG_W * self.S), int(IMG_H * self.S)
        self.W, self.H = self.iw + PANEL_W, self.ih + BOTTOM_H
        self.screen = pygame.display.set_mode((self.W, self.H))
        pygame.display.set_caption(NOME_PROJETO + " - Robô Móvel Inteligente de Vigilância (IMR)")
        self.clock = pygame.time.Clock()
        fnt = "dejavusansmono,consolas,menlo,couriernew,monospace"
        self.f = pygame.font.SysFont(fnt, 13)
        self.fb = pygame.font.SysFont(fnt, 15, bold=True)
        self.fs = pygame.font.SysFont(fnt, 11)
        self.ft = pygame.font.SysFont(fnt, 19, bold=True)
        self.sim = Sim()
        self.bg = None
        path = os.path.join(os.path.dirname(os.path.abspath(__file__)), IMG_FILE)
        if os.path.exists(path):
            img = pygame.image.load(path).convert()
            self.bg = pygame.transform.smoothscale(img, (self.iw, self.ih))
        self.view = 0               # 0 planta | 1 mapa aprendido | 2 grade inflada
        self.show_lidar, self.show_barriers, self.show_odom = True, False, True
        self.paused, self.speed = False, 1
        self.map_surf, self.map_t = None, -9.0
        self.mouse_world = None

    # --- conversões tela <-> planta
    def p2s(self, px, py):
        return int(px * self.S), int(py * self.S)

    def m2s(self, x, y):
        return self.p2s(*m2px(x, y))

    def s2p(self, sx, sy):
        return sx / self.S, sy / self.S

    # --- mapa aprendido como imagem
    def learned_surface(self):
        sim = self.sim
        if sim.t - self.map_t < 0.25 and self.map_surf is not None:
            return self.map_surf
        self.map_t = sim.t
        mp = sim.mapper
        img = np.empty((GR, GC, 3), np.uint8)
        img[:] = (95, 100, 112)                                   # desconhecido
        img[mp.free()] = (236, 234, 224)                          # livre
        occ = mp.occupied()
        img[occ] = (24, 24, 30)                                   # ocupado
        novos = occ & ~sim.planner.static_near
        img[novos] = (255, 140, 0)                                # obstáculo novo
        self.map_surf = pygame.surfarray.make_surface(img.swapaxes(0, 1))
        return self.map_surf

    # ------------------------------------------------------------ desenho
    def draw_world(self):
        sc, sim = self.screen, self.sim
        if self.view == 1:
            m = pygame.transform.scale(self.learned_surface(), (self.iw, self.ih))
            sc.blit(m, (0, 0))
        else:
            if self.bg:
                sc.blit(self.bg, (0, 0))
            else:
                sc.fill((226, 205, 180), (0, 0, self.iw, self.ih))
                for _, x0, y0, x1, y1 in PAREDES + MOVEIS:
                    pygame.draw.rect(sc, (90, 70, 60), (*self.p2s(x0, y0), int((x1 - x0) * self.S),
                                                        int((y1 - y0) * self.S)))
                for _, pts in PAREDES_POLI:
                    pygame.draw.polygon(sc, (90, 70, 60), [self.p2s(*q) for q in pts])
        if self.view == 2:
            ov = pygame.Surface((self.iw, self.ih), pygame.SRCALPHA)
            ib = pygame.transform.scale(self._grid_surface(sim.planner.static_blocked, (255, 60, 60, 110)),
                                        (self.iw, self.ih))
            ov.blit(ib, (0, 0))
            if sim.planner.new_obs.any():
                nb = dilate(sim.planner.new_obs, INFLATE_CELLS)
                ov.blit(pygame.transform.scale(self._grid_surface(nb, (255, 160, 0, 150)),
                                               (self.iw, self.ih)), (0, 0))
            sc.blit(ov, (0, 0))
        if self.show_barriers:
            ov = pygame.Surface((self.iw, self.ih), pygame.SRCALPHA)
            for _, x0, y0, x1, y1 in PAREDES + MOVEIS:
                r = pygame.Rect(*self.p2s(x0, y0), max(1, int((x1 - x0) * self.S)), max(1, int((y1 - y0) * self.S)))
                pygame.draw.rect(ov, (255, 0, 0, 70), r)
                pygame.draw.rect(ov, (255, 60, 60, 200), r, 1)
            for _, pts in PAREDES_POLI:
                q = [self.p2s(*pt) for pt in pts]
                pygame.draw.polygon(ov, (255, 0, 0, 70), q)
                pygame.draw.polygon(ov, (255, 60, 60, 200), q, 1)
            sc.blit(ov, (0, 0))
        # intruso simulado (verdade do simulador) e visão noturna
        if sim.intruder.on:
            ip = self.m2s(sim.intruder.x, sim.intruder.y)
            pygame.draw.circle(sc, (230, 60, 200), ip, max(4, int(INTRUSO_R * PX_PER_M * self.S)))
            pygame.draw.circle(sc, (255, 255, 255), ip, max(4, int(INTRUSO_R * PX_PER_M * self.S)), 1)
        if sim.p.night:
            ov = pygame.Surface((self.iw, self.ih), pygame.SRCALPHA)
            ov.fill((8, 12, 40, 175))
            rr = self.m2s(sim.robot.x, sim.robot.y)
            pygame.draw.circle(ov, (0, 0, 0, 0), rr, int(1.6 * PX_PER_M * self.S))
            sc.blit(ov, (0, 0))
        # portas
        for x, y0, y1 in PORTAS:
            pygame.draw.line(sc, (80, 230, 140), self.p2s(x, y0), self.p2s(x, y1), 3)
        # caixas surpresa
        for r in sim.world.boxes:
            rr = pygame.Rect(*self.p2s(r.left, r.top), int(r.w * self.S), int(r.h * self.S))
            pygame.draw.rect(sc, (210, 120, 40), rr)
            pygame.draw.rect(sc, (255, 220, 160), rr, 2)
            sc.blit(self.fs.render("caixa", True, (40, 20, 0)), (rr.x + 3, rr.centery - 6))
        # cômodos (rótulos dos destinos)
        for n, (px, py) in COMODOS.items():
            pos = self.p2s(px, py)
            pygame.draw.circle(sc, (255, 255, 255), pos, 4, 1)
            t = self.fs.render(n, True, (255, 255, 255), (30, 30, 40))
            sc.blit(t, (pos[0] - t.get_width() // 2, pos[1] + 6))
        # trilhas
        if len(sim.trail) > 1:
            pygame.draw.lines(sc, (80, 230, 120), False, [self.m2s(*p) for p in sim.trail], 2)
        if self.show_odom and len(sim.odom_trail) > 1:
            pygame.draw.lines(sc, (255, 130, 30), False, [self.m2s(*p) for p in sim.odom_trail], 1)
        # caminho planejado
        pts = sim.brain.path
        if len(pts) > 1:
            sp = [self.m2s(*p) for p in pts]
            pygame.draw.lines(sc, (0, 220, 255), False, sp, 3)
            for i, q in enumerate(sp):
                pygame.draw.circle(sc, (0, 255, 255) if i >= sim.brain.idx else (0, 120, 140), q, 4)
            pygame.draw.circle(sc, (255, 215, 0), sp[-1], 9, 3)
        # lidar
        r = sim.robot
        rp = self.m2s(r.x, r.y)
        scan = sim.bus.last.get("/scan")
        if self.show_lidar and scan is not None:
            for k in range(0, 360, 2):
                d = scan[k]
                if d == 0.0:
                    continue
                dd = LIDAR_RANGE if not np.isfinite(d) else d
                a = r.th + math.radians(k)
                e = self.m2s(r.x + dd * math.cos(a), r.y + dd * math.sin(a))
                hit = np.isfinite(d) and d < LIDAR_RANGE
                pygame.draw.line(sc, (255, 210, 60) if hit else (60, 200, 120), rp, e, 1)
                if hit:
                    pygame.draw.circle(sc, (255, 80, 60), e, 2)
        # odometria (fantasma) e robô real
        if self.show_odom:
            o = sim.odom
            op = self.m2s(o.x, o.y)
            pygame.draw.circle(sc, (255, 130, 30), op, max(4, int(ROBOT_R * PX_PER_M * self.S)), 2)
            pygame.draw.line(sc, (255, 130, 30), op, (op[0] + 16 * math.cos(o.th) * self.S * 1.2,
                                                       op[1] - 16 * math.sin(o.th) * self.S * 1.2), 2)
        rad = max(5, int(ROBOT_R * PX_PER_M * self.S))
        pygame.draw.circle(sc, (0, 170, 255), rp, rad)
        pygame.draw.circle(sc, (255, 255, 255), rp, rad, 2)
        fx, fy = rp[0] + (rad + 6) * math.cos(r.th), rp[1] - (rad + 6) * math.sin(r.th)
        pygame.draw.line(sc, (255, 60, 60), rp, (fx, fy), 3)
        # alvos detectados pela vigilância (LiDAR)
        for tr in sim.security.tracks:
            tp = self.m2s(*tr["pos"])
            col = (255, 60, 60) if tr["mov"] else (255, 170, 40)
            pygame.draw.circle(sc, col, tp, 16, 2)
            lab = "MOVIMENTO" if tr["mov"] else "objeto novo"
            t = self.fs.render(f"#{tr['id']} {lab}", True, (0, 0, 0), col)
            sc.blit(t, (tp[0] - t.get_width() // 2, tp[1] - 32))
        # rótulo do estado em cima do robô
        est = sim.brain.estado
        t = self.fs.render(est, True, (0, 0, 0), COR_ESTADO.get(est, (200, 200, 200)))
        sc.blit(t, (rp[0] - t.get_width() // 2, rp[1] - rad - 18))
        if self.view == 1:
            sc.blit(self.fb.render("MAPA APRENDIDO (grade de ocupação)", True, (255, 255, 255), (30, 30, 40)), (8, 8))
        elif self.view == 2:
            sc.blit(self.fb.render("GRADE INFLADA p/ planejamento A*", True, (255, 255, 255), (30, 30, 40)), (8, 8))

    def draw_hud(self):
        sc, sim = self.screen, self.sim
        sec = sim.security
        w, h = 330, 26 + 16 * max(1, min(4, len(sec.alertas)))
        x0, y0 = self.iw - w - 8, 8
        box = pygame.Surface((w, h), pygame.SRCALPHA)
        box.fill((10, 12, 20, 205))
        sc.blit(box, (x0, y0))
        noite = sim.p.night
        pygame.draw.rect(sc, (255, 90, 90) if sec.tracks and noite else (90, 120, 255) if noite else (120, 200, 130),
                         (x0, y0, w, 3))
        self.text(f"VIGILÂNCIA  {'● NOITE' if noite else '○ DIA'}   Intruso: "
                  f"{'ON' if sim.intruder.on else 'off'}   Alertas: {sec.n_alertas}", x0 + 8, y0 + 7,
                  (255, 255, 255), self.fs)
        y = y0 + 24
        ult = list(sec.alertas)[-4:]
        if not ult:
            self.text("sem alertas  (T: noturno, I: intruso de teste)", x0 + 8, y, (150, 160, 175), self.fs)
        for a in reversed(ult):
            col = (255, 110, 110) if a["nivel"] == "CRITICO" else (255, 200, 90)
            self.text(f"{a['tempo_sim']:6.1f}s {a['tipo'].replace('_', ' '):<18} {a['comodo']}", x0 + 8, y, col, self.fs)
            y += 16

    def _grid_surface(self, grid, rgba):
        arr = np.zeros((GR, GC, 4), np.uint8)
        arr[grid] = rgba
        s = pygame.Surface((GC, GR), pygame.SRCALPHA)
        pygame.surfarray.pixels3d(s)[:] = arr[..., :3].swapaxes(0, 1)
        pygame.surfarray.pixels_alpha(s)[:] = arr[..., 3].swapaxes(0, 1)
        return s

    def text(self, s, x, y, col=(225, 225, 230), font=None):
        self.screen.blit((font or self.f).render(s, True, col), (x, y))

    def draw_panel(self):
        sc, sim = self.screen, self.sim
        x0 = self.iw
        pygame.draw.rect(sc, (24, 26, 32), (x0, 0, PANEL_W, self.H))
        y = 8
        self.text("vigiLAR  ·  CDC.6NA", x0 + 12, y, (255, 255, 255), self.ft)
        y += 30
        est = sim.brain.estado
        pygame.draw.rect(sc, COR_ESTADO.get(est, (150, 150, 150)), (x0 + 12, y, PANEL_W - 24, 26), border_radius=5)
        self.text("FSM: " + est, x0 + 20, y + 5, (10, 10, 10), self.fb)
        y += 34
        loc = "ODOMETRIA (deriva!)" if sim.p.use_odom else "ground truth"
        self.text(f"Modo: {sim.p.mode:<9} Loc.: {loc}", x0 + 12, y)
        y += 16
        self.text(f"Ruído: {'ON' if sim.p.noise else 'OFF'}   Sim: {self.speed}x"
                  f"{'  [PAUSADO]' if self.paused else ''}", x0 + 12, y)
        y += 22
        r, o = sim.robot, sim.odom
        err, derr = sim.odom_error()
        self.text("TELEMETRIA", x0 + 12, y, (130, 200, 255), self.fb)
        y += 18
        for ln in (
            f"Real  x={r.x:5.2f} y={r.y:5.2f} θ={math.degrees(r.th):6.1f}°",
            f"Odom  x={o.x:5.2f} y={o.y:5.2f} θ={math.degrees(o.th):6.1f}°",
            f"Erro odom: {err:5.2f} m  {derr:+6.1f}°",
            f"v={r.v:5.2f} m/s  ω={r.w:5.2f} rad/s",
            f"roda E={r.vl:5.2f}  D={r.vr:5.2f} (lim {MAX_WHEEL})",
            f"Distância: {r.dist:5.1f} m   Colisões: {r.collisions}",
        ):
            self.text(ln, x0 + 12, y)
            y += 15
        y += 8
        self.text("LiDAR (setores filtrados)", x0 + 12, y, (130, 200, 255), self.fb)
        y += 18
        sec = sim.brain.sec
        for nome in ("esquerda", "frente", "direita"):
            d = sec[nome]
            w = int((PANEL_W - 130) * min(d, 3.0) / 3.0)
            col = (240, 80, 80) if d < D_CRITICA else (255, 200, 60) if d < 0.8 else (80, 220, 130)
            self.text(f"{nome[:6]:<6}{d:4.2f}", x0 + 12, y)
            pygame.draw.rect(sc, (50, 52, 60), (x0 + 112, y + 2, PANEL_W - 130, 10))
            pygame.draw.rect(sc, col, (x0 + 112, y + 2, w, 10))
            y += 15
        y += 8
        self.text("TÓPICOS (estilo ROS 2)", x0 + 12, y, (130, 200, 255), self.fb)
        y += 18
        for tp in ("/scan", "/scan_sectors", "/odom", "/ground_truth", "/path", "/cmd_vel", "/wheel_cmd", "/map", "/alerta"):
            hz = sim.bus.rate.get(tp, 0.0)
            fresh = (sim.bus.now - sim.bus.stamp.get(tp, -9)) < 0.15
            pygame.draw.circle(sc, (90, 240, 140) if fresh else (70, 74, 82), (x0 + 18, y + 7), 4)
            msg = sim.bus.last.get(tp)
            extra = ""
            if tp == "/cmd_vel" and msg:
                extra = f"v={msg[0]:.2f} ω={msg[1]:.2f}"
            elif tp == "/scan" and msg is not None:
                extra = "360 raios"
            elif tp == "/path" and msg:
                extra = f"{len(msg['pts'])} wp"
            elif tp == "/map" and msg:
                extra = f"cob={msg['coverage'] * 100:.0f}%"
            elif tp == "/alerta" and msg:
                extra = msg["tipo"][:14]
            self.text(f"{tp:<14}{hz:5.1f}Hz {extra}", x0 + 28, y, (200, 200, 208), self.fs)
            y += 14
        y += 8
        # minimapa aprendido
        mh = int((PANEL_W - 24) * MAPH / MAPW)
        if y + mh + 20 < self.H - 4:
            self.text(f"MAPA APRENDIDO  cobertura {sim.mapper.coverage * 100:3.0f}%", x0 + 12, y,
                      (130, 200, 255), self.fb)
            y += 18
            m = pygame.transform.smoothscale(self.learned_surface(), (PANEL_W - 24, mh))
            sc.blit(m, (x0 + 12, y))
            pygame.draw.rect(sc, (90, 90, 100), (x0 + 12, y, PANEL_W - 24, mh), 1)
            k = (PANEL_W - 24) / MAPW
            px, py = m2px(r.x, r.y)
            pygame.draw.circle(sc, (0, 170, 255), (int(x0 + 12 + px * k), int(y + py * k)), 4)

    def draw_bottom(self):
        sc = self.screen
        pygame.draw.rect(sc, (16, 18, 22), (0, self.ih, self.iw, BOTTOM_H))
        y = self.ih + 4
        for ln in list(self.sim.logs)[-3:]:
            self.text(ln, 8, y, (255, 225, 140), self.fs)
            y += 13
        y += 3
        for ln in HELP:
            self.text(ln, 8, y, (150, 160, 175), self.fs)
            y += 12

    # ------------------------------------------------------------ eventos
    def handle(self, e):
        sim = self.sim
        if e.type == pygame.QUIT:
            return False
        if e.type == pygame.MOUSEBUTTONDOWN and e.pos[0] < self.iw and e.pos[1] < self.ih:
            px, py = self.s2p(*e.pos)
            if e.button == 1:
                if 57 < px < 1218 and 33 < py < 690:
                    sim.go_to_px(px, py)
            elif e.button == 3:
                res = sim.world.toggle_box(px, py, (sim.robot.x, sim.robot.y))
                msgs = {"criada": "Caixa surpresa colocada! (o robô vai perceber pelo LiDAR)",
                        "removida": "Caixa removida", "ocupado": "Aí já tem móvel/parede",
                        "perto": "Muito perto do robô"}
                sim.log(msgs[res])
        elif e.type == pygame.KEYDOWN:
            k = e.key
            names = list(COMODOS.keys())
            if k == pygame.K_ESCAPE:
                return False
            elif pygame.K_1 <= k <= pygame.K_6:
                sim.go_to_room(names[k - pygame.K_1])
            elif k == pygame.K_p:
                sim.p.mode = "PATRULHA"
                sim.brain.estado = "PARADO" if sim.brain.estado == "MANUAL" else sim.brain.estado
                sim.log("Modo patrulha: Cozinha > Jantar > Sala > Banheiro > Quarto > Entrada")
            elif k == pygame.K_t:
                sim.p.night = not sim.p.night
                if sim.p.night:
                    if sim.p.mode != "MANUAL":
                        sim.p.mode = "PATRULHA"
                    sim.log("MODO NOTURNO: ronda autônoma + vigilância de movimento (alertas ativos)")
                else:
                    sim.log("Modo diurno: movimentos são apenas registrados, sem alerta")
            elif k == pygame.K_i:
                r = sim.intruder.toggle()
                sim.log({True: "Intruso de teste entrou pela Entrada", False: "Intruso removido",
                         None: "Sem cômodo alcançável para o intruso"}[r])
            elif k == pygame.K_e:
                sim.p.mode = "EXPLORAR"
                sim.mission.blacklist, sim.mission.cur_frontier = [], None
                sim.brain.estado = "PARADO"
                sim.log("Exploração autônoma por fronteiras (sem usar o mapa pronto)")
            elif k == pygame.K_TAB:
                if sim.p.mode == "MANUAL":
                    sim.stop()
                    sim.log("Modo automático")
                else:
                    sim.p.mode = "MANUAL"
                    sim.planner.clear()
                    sim.brain.path = []
                    sim.log("Modo MANUAL: use as setas")
            elif k == pygame.K_s:
                sim.stop()
                sim.log("Parado")
            elif k == pygame.K_v:
                self.view = (self.view + 1) % 3
            elif k == pygame.K_l:
                self.show_lidar = not self.show_lidar
            elif k == pygame.K_b:
                self.show_barriers = not self.show_barriers
            elif k == pygame.K_o:
                self.show_odom = not self.show_odom
            elif k == pygame.K_u:
                sim.p.use_odom = not sim.p.use_odom
                sim.log("Guiando pela ODOMETRIA (vai derivar)" if sim.p.use_odom else "Guiando pelo ground truth")
            elif k == pygame.K_z:
                sim.odom.relocalize()
                sim.log("Odometria relocalizada (reset da deriva)")
            elif k == pygame.K_n:
                sim.p.noise = not sim.p.noise
                sim.log("Ruído " + ("LIGADO" if sim.p.noise else "DESLIGADO"))
            elif k == pygame.K_SPACE:
                self.paused = not self.paused
            elif k == pygame.K_RIGHTBRACKET:
                self.speed = min(8, self.speed * 2)
            elif k == pygame.K_LEFTBRACKET:
                self.speed = max(1, self.speed // 2)
            elif k == pygame.K_r:
                sim.reset()
                self.map_surf = None
        return True

    def manual_keys(self):
        keys = pygame.key.get_pressed()
        v = w = 0.0
        if keys[pygame.K_UP]:
            v += 0.6
        if keys[pygame.K_DOWN]:
            v -= 0.4
        if keys[pygame.K_LEFT]:
            w += 1.8
        if keys[pygame.K_RIGHT]:
            w -= 1.8
        self.sim.p.manual = (v, w)

    def run(self):
        running = True
        while running:
            for e in pygame.event.get():
                running = self.handle(e) and running
            if self.sim.p.mode == "MANUAL":
                self.manual_keys()
            if not self.paused:
                for _ in range(self.speed):
                    self.sim.step()
            self.screen.fill((0, 0, 0))
            self.draw_world()
            self.draw_hud()
            self.draw_panel()
            self.draw_bottom()
            pygame.display.flip()
            self.clock.tick(60)
        pygame.quit()


def main():
    App().run()


if __name__ == "__main__":
    main()
