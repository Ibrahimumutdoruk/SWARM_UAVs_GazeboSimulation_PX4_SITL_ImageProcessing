#!/usr/bin/env python3
"""
Madde 1 regresyon araci. Algoritmaya DOKUNMAZ; sadece gozlemler.

/swarm/agent_state dinler ve:
  1) Her UAV'nin FSM state gecislerini zaman damgasiyla CSV'ye yazar.
  2) Her ciftin global mesafesini izleyip minimum ayrimi (collision metrigi) raporlar.
  3) QR/renk olaylarini (varsa /uavN/vision/* ile) ister.

Kullanim:
  source ~/swarm_ws/install/setup.bash
  python3 regression_logger.py            # ./regression_<ts>.csv yazar
Cikti: konsola canli gecis + sonda OZET (sira + min mesafe + DONE'a ulasildi mi).

Kabul kriteri (Madde 1): bu logger calistirilip nominal akis tekrar uretildiginde
ayni FSM sirasi gorulmeli ve min mesafe >= d_safe olmali. Sonraki her maddeden sonra
bu ayni kosturulup nominal akisin bozulmadigi dogrulanir (regresyon).
"""
import csv, time, itertools, math
from datetime import datetime
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from laplacian_interfaces.msg import AgentState

FSM = {0:'IDLE',1:'SYNC',2:'TAKEOFF',3:'NAV',4:'READ',5:'EXECUTE',
       6:'SPLIT',7:'RTL',8:'DONE',9:'BARRIER'}

class RegLogger(Node):
    def __init__(self):
        super().__init__('regression_logger')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST, depth=20)
        self.create_subscription(AgentState, '/swarm/agent_state', self.cb, qos)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.path = f'regression_{ts}.csv'
        self.f = open(self.path, 'w', newline='')
        self.w = csv.writer(self.f)
        self.w.writerow(['t_rel','uav_id','fsm','event'])
        self.t0 = time.time()
        self.last = {}                 # uav -> fsm int
        self.pos = {}                  # uav -> (E,N,U)
        self.seq = {}                  # uav -> [state names in order]
        self.min_sep = float('inf')
        self.min_sep_t = None
        self.create_timer(0.2, self.sep_check)
        self.get_logger().info(f'logging -> {self.path}  (Ctrl-C to stop + summary)')

    def cb(self, m):
        self.pos[m.drone_id] = tuple(m.position)
        prev = self.last.get(m.drone_id)
        if prev != m.fsm_state:
            self.last[m.drone_id] = m.fsm_state
            name = FSM.get(m.fsm_state, str(m.fsm_state))
            self.seq.setdefault(m.drone_id, []).append(name)
            t = time.time() - self.t0
            self.w.writerow([f'{t:.2f}', m.drone_id, name, 'state_change'])
            self.f.flush()
            self.get_logger().info(f'[{t:6.2f}s] uav{m.drone_id} -> {name}')

    def sep_check(self):
        ids = sorted(self.pos)
        for a, b in itertools.combinations(ids, 2):
            pa, pb = self.pos[a], self.pos[b]
            d = math.dist(pa, pb)
            if d < self.min_sep:
                self.min_sep = d
                self.min_sep_t = time.time() - self.t0

    def summary(self):
        print('\n================ REGRESSION SUMMARY ================')
        print(f'csv: {self.path}')
        for uid in sorted(self.seq):
            reached = 'DONE' in self.seq[uid]
            print(f'uav{uid} seq: {" -> ".join(self.seq[uid])}   reached_DONE={reached}')
        sep = 'n/a' if self.min_sep == float('inf') else f'{self.min_sep:.2f} m @ t={self.min_sep_t:.1f}s'
        print(f'min pairwise separation (global): {sep}')
        print('PASS hint: every uav ends at DONE AND min separation >= configured d_safe')
        print('====================================================\n')
        self.f.close()

def main():
    rclpy.init()
    n = RegLogger()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.summary()
        n.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
