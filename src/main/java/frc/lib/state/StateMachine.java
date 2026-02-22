package frc.lib.state;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

public class StateMachine <E extends Enum<E>> {
    private E currentState;
    private final Map<E, StateNode<E>> stateNodes;

    // Guard fields to prevent multiple transitions within a single tick
    private boolean inTick = false;
    private boolean transitionFiredThisTick = false;

    public StateMachine(E initialState, Map<E, StateNode<E>> stateNodes) {
        this.currentState = initialState;
        this.stateNodes = stateNodes;
        stateNodes.get(currentState).onEnter.run();
    }

    public static <E extends Enum<E>> Builder<E> forEnum() {
        return new Builder<>();
    }

    private StateNode<E> state(E e) {
        StateNode<E> def = stateNodes.get(e);
        if (def == null) throw new IllegalStateException("State not defined: " + e);
        return def;
    }

    public static final class Builder<E extends Enum<E>> {
        private E initial;
        private final Map<E, StateNode<E>> states = new HashMap<>();

        public Builder<E> initial(E initial) {
            this.initial = initial;
            return this;
        }

        public Builder<E> state(E id, Consumer<StateConfigurer<E>> cfg) {
            StateNode<E> def = states.computeIfAbsent(id, k -> new StateNode<>());
            cfg.accept(new StateConfigurer<>(def));
            return this;
        }

        public StateMachine<E> build() {
            if (initial == null) throw new IllegalStateException("initial(...) is required");
            if (!states.containsKey(initial))
                throw new IllegalStateException("Initial state not defined: " + initial);
            return new StateMachine<>(initial, states);
        }
    }

    public static final class StateConfigurer<S extends Enum<S>> {
        private final StateNode<S> def;

        StateConfigurer(StateNode<S> def) { this.def = def; }

        public StateConfigurer<S> onEnter(Runnable r) { def.onEnter = r; return this; }
        public StateConfigurer<S> onExit(Runnable r)  { def.onExit = r;  return this; }
        public StateConfigurer<S> onTick(Runnable r)  { def.onTick = r;  return this; }

        public TransitionConfigurer<S> transitionTo(S target) {
            Transition<S> t = new Transition<>(target);
            def.transitions.add(t);
            return new TransitionConfigurer<>(this, t);
        }
    }

    public static final class TransitionConfigurer<S extends Enum<S>> {
        private final StateConfigurer<S> parent;
        private final Transition<S> t;

        TransitionConfigurer(StateConfigurer<S> parent, Transition<S> t) {
            this.parent = parent;
            this.t = t;
        }

        public StateConfigurer<S> when(BooleanSupplier guard) {
            t.guard = guard;
            return parent; // return to state configurer for chaining
        }
    }

    public void tick() {
        // prevent re-entrant ticks
        if (inTick) return;

        inTick = true;
        transitionFiredThisTick = false;
        try {
            StateNode<E> stateNode = stateNodes.get(currentState);
            for (Transition<E> transition : stateNode.transitions) {
                if (transition.guard.getAsBoolean()) {
                    // mark that a transition has fired for this tick before running enter/exit
                    transitionFiredThisTick = true;
                    stateNode.onExit.run();
                    currentState = transition.target;
                    stateNodes.get(currentState).onEnter.run();
                    return;
                }
            }
            stateNode.onTick.run();
        } finally {
            // reset guards when tick completes
            inTick = false;
            transitionFiredThisTick = false;
        }
    }

    public void transitionTo(E targetState) {
        // If called during a tick and a transition already fired, ignore to avoid cascading transitions
        if (inTick && transitionFiredThisTick) {
            return;
        }
        // If called during tick and no transition has fired yet, mark one now
        if (inTick) transitionFiredThisTick = true;

        StateNode<E> stateNode = stateNodes.get(currentState);
        stateNode.onExit.run();
        currentState = targetState;
        stateNodes.get(currentState).onEnter.run();
    }

    public E getCurrentState() {
        return currentState;
    }

    public static final class StateNode<E> {
        Runnable onEnter = () -> {};
        Runnable onExit = () -> {};
        Runnable onTick = () -> {};
        List<Transition<E>> transitions = new ArrayList<>();
    }

    public static final class Transition<E> {
        final E target;
        BooleanSupplier guard = () -> true;

        Transition(E target) { this.target = target; }
    }

}
