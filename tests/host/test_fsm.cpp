/**
 * @file
 */

#include <cstdint>
#include <cstdio>

#include "micras/core/fsm.hpp"
#include "test_host.hpp"

using namespace micras::core;

namespace {
struct Counting : FsmState {
    Counting(uint8_t id, uint8_t next) : FsmState{id}, next{next} { }

    void on_entry() override { entries++; }

    uint8_t execute() override {
        executions++;
        return next;
    }

    uint8_t next;
    int     entries{};
    int     executions{};
};
}  // namespace

int main() {
    Counting idle{0, 0};
    Counting run{1, 0};
    Counting wait{2, 1};

    TFsm<3> fsm{0};
    fsm.add_state(idle);
    fsm.add_state(run);
    fsm.add_state(wait);

    // --- the initial state is only chosen until the first update enters it ---
    CHECK(not fsm.has_entered_current_state());
    fsm.update();
    CHECK(fsm.has_entered_current_state());
    CHECK(idle.entries == 1 && idle.executions == 1);

    // --- a state chosen from outside is entered by the next update, with its entry function ---
    fsm.transition_to(2);
    CHECK(fsm.get_current_state_id() == 2 && not fsm.has_entered_current_state());
    fsm.update();
    CHECK(wait.entries == 1 && wait.executions == 1);

    // --- a state chosen by the one before it is not entered until the next update ---
    CHECK(fsm.get_current_state_id() == 1 && not fsm.has_entered_current_state());
    fsm.update();
    CHECK(run.entries == 1);
    CHECK(fsm.get_current_state_id() == 0 && not fsm.has_entered_current_state());

    // --- choosing another state before it is entered skips its entry function ---
    fsm.transition_to(2);
    fsm.update();
    CHECK(idle.entries == 1 && wait.entries == 2);

    // --- staying in a state does not enter it again ---
    fsm.update();
    fsm.transition_to(0);
    fsm.update();
    fsm.update();
    CHECK(idle.entries == 2 && fsm.has_entered_current_state());

    std::puts("fsm ok");
}
