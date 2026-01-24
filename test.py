import time
import cereal.messaging as messaging
from cereal import log

EVENT_ENUM = {v: k for k, v in log.OnroadEvent.EventName.schema.enumerants.items()}

def ev_name(ev):
  return EVENT_ENUM.get(ev.name, str(ev.name))

def ev_flags(ev):
  flags = []
  for k in ("noEntry", "softDisable", "immediateDisable", "userDisable",
            "warning", "preEnable", "overrideLateral", "overrideLongitudinal",
            "permanent", "enable"):
    if getattr(ev, k):
      flags.append(k)
  return ",".join(flags)

sm = messaging.SubMaster([
  "selfdriveState", "carState", "carControl", "onroadEvents", "managerState"
])

last_sig = None
while True:
  sm.update(100)

  cs = sm["carState"]
  ss = sm["selfdriveState"]
  cc = sm["carControl"]
  events = list(sm["onroadEvents"])

  blocked = [f"{ev_name(e)}({ev_flags(e)})" for e in events
             if e.noEntry or e.softDisable or e.immediateDisable]
  not_running = [p.name for p in sm["managerState"].processes if p.shouldBeRunning and not p.running]

  sig = (cs.cruiseState.enabled, ss.enabled, ss.active, cc.latActive, cc.longActive,
         tuple(blocked), tuple(not_running))
  if sig != last_sig:
    print(
      f"AI={cs.cruiseState.enabled} enabled={ss.enabled} active={ss.active} "
      f"latActive={cc.latActive} longActive={cc.longActive} vEgo={cs.vEgo:.2f} "
      f"blocked={blocked} not_running={not_running}"
    )
    last_sig = sig

  time.sleep(0.1)
