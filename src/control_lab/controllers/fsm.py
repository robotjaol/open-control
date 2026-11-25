"""Finite state machine with a transition table, Moore outputs and structural checks.

The machine is a deterministic Moore machine: outputs depend on the current state only. An event
with no entry for the current state is recorded as dropped (a "missed event"), never silently
accepted. Structural checks (reachability, deadlock) work on the table alone.
"""

from __future__ import annotations

from collections import deque


class FSM:
    def __init__(self, initial: str, transitions: dict, outputs: dict | None = None):
        """transitions: {(state, event): next_state}. outputs: {state: {name: value}}."""
        self.initial = initial
        self.transitions = dict(transitions)
        self.outputs = outputs or {}
        self.states = sorted({s for s, _ in self.transitions} | set(self.transitions.values())
                             | {initial})
        self.events = sorted({e for _, e in self.transitions})
        for s in self.states:
            self.outputs.setdefault(s, {})
        self.reset()

    def reset(self) -> None:
        self.state = self.initial
        self.log: list[tuple] = []
        self.dropped: list[tuple] = []

    def fire(self, event: str, t: float = 0.0) -> bool:
        """Apply an event. Returns True if a transition was taken."""
        key = (self.state, event)
        if key not in self.transitions:
            self.dropped.append((t, self.state, event))
            return False
        nxt = self.transitions[key]
        self.log.append((t, self.state, event, nxt))
        self.state = nxt
        return True

    def output(self, name: str, default=0):
        return self.outputs[self.state].get(name, default)

    # structural analysis -------------------------------------------------------------------
    def successors(self, state: str) -> set:
        return {nxt for (s, _), nxt in self.transitions.items() if s == state}

    def reachable_from(self, start: str) -> set:
        seen, queue = {start}, deque([start])
        while queue:
            for nxt in self.successors(queue.popleft()):
                if nxt not in seen:
                    seen.add(nxt)
                    queue.append(nxt)
        return seen

    def can_reach(self, start: str, target: str) -> bool:
        return target in self.reachable_from(start)

    def unreachable_states(self) -> set:
        return set(self.states) - self.reachable_from(self.initial)

    def deadlock_states(self) -> set:
        """States with no outgoing transition to a different state."""
        return {s for s in self.states if not (self.successors(s) - {s})}

    def to_table(self) -> list[dict]:
        return [{"state": s, "event": e, "next": n} for (s, e), n in sorted(self.transitions.items())]

    def to_dot(self) -> str:
        lines = ["digraph fsm {", "  rankdir=LR;"]
        for (s, e), n in sorted(self.transitions.items()):
            lines.append(f'  "{s}" -> "{n}" [label="{e}"];')
        lines.append("}")
        return "\n".join(lines)
