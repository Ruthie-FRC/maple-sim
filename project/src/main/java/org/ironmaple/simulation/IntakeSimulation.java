package org.ironmaple.simulation;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Distance;
import frcsim_physics.BodyFixture2d;
import frcsim_physics.CollisionData2d;
import frcsim_physics.Contact2d;
import frcsim_physics.ContactListener2d;
import frcsim_physics.Convex2d;
import frcsim_physics.RectangleShape;
import frcsim_physics.RigidBody;
import frcsim_physics.SolvedContact2d;
import frcsim_physics.Vec2;
import java.util.ArrayDeque;
import java.util.Objects;
import java.util.Queue;
import java.util.function.Predicate;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralAlgaeStack;

/**
 *
 *
 * <h2>Simulates an Intake Mechanism on the Robot.</h2>
 *
 * <p>Check<a href='https://shenzhen-robotics-alliance.github.io/maple-sim/simulating-intake/'>Online Documentation</a>
 *
 * <p>The intake is a 2D component attached to one side of the robot's chassis. It is rectangular in shape and extends
 * from the robot when activated.
 *
 * <p>The intake can be turned on through {@link #startIntake()}, which causes it to extend, expanding the collision
 * space of the robot's chassis. When turned off via {@link #stopIntake()}, the intake retracts.
 *
 * <p>The intake can "collect" {@link GamePieceOnFieldSimulation} instances from the field, removing them and
 * incrementing the {@link #gamePiecesInIntakeCount}.
 *
 * <p>A game piece is collected if the following conditions are met:
 *
 * <ul>
 *   <li>1. The type of the game piece ({@link org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation#type})
 *       matches {@link #targetedGamePieceType}.
 *   <li>2. The {@link org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation} is in contact with the intake
 *       (and not other parts of the robot).
 *   <li>3. The intake is turned on via {@link #startIntake()}.
 *   <li>4. The number of game pieces in the intake ({@link #gamePiecesInIntakeCount}) is less than {@link #capacity}.
 * </ul>
 *
 * <p><strong>Note:</strong> This class simulates an idealized "touch it, get it" intake and does not model the actual
 * functioning of an intake mechanism.
 */
public class IntakeSimulation extends BodyFixture2d {
    private final int capacity;
    private int gamePiecesInIntakeCount;
    private boolean intakeRunning;

    private final Queue<GamePieceOnFieldSimulation> gamePiecesToRemove;
    private final AbstractDriveTrainSimulation driveTrainSimulation;
    private final String targetedGamePieceType;
    private Predicate<GamePieceOnFieldSimulation> customIntakeCondition = gp -> true;

    public enum IntakeSide {
        FRONT,
        LEFT,
        RIGHT,
        BACK
    }

    /**
     *
     *
     * <h2>Creates an Intake Simulation that Tightly Attaches to One Side of the Chassis.</h2>
     *
     * <p>This typically represents an In-The-Frame (ITF) Intake.
     *
     * @param targetedGamePieceType the type of game pieces that this intake can collect
     * @param driveTrainSimulation the chassis to which this intake is attached
     * @param width the width of the intake
     * @param side the side of the chassis where the intake is attached
     * @param capacity the maximum number of game pieces that the intake can hold
     */
    public static IntakeSimulation InTheFrameIntake(
            String targetedGamePieceType,
            AbstractDriveTrainSimulation driveTrainSimulation,
            Distance width,
            IntakeSide side,
            int capacity) {
        return OverTheBumperIntake(targetedGamePieceType, driveTrainSimulation, width, Meters.of(0.02), side, capacity);
    }

    /**
     *
     *
     * <h2>Creates an Intake Simulation that Extends Out of the Chassis Frame.</h2>
     *
     * <p>This typically represents an Over-The-Bumper (OTB) Intake.
     *
     * @param targetedGamePieceType the type of game pieces that this intake can collect
     * @param driveTrainSimulation the chassis to which this intake is attached
     * @param width the valid width of the intake
     * @param lengthExtended the length the intake extends out from the chassis when activated
     * @param side the side of the chassis where the intake is attached
     * @param capacity the maximum number of game pieces that the intake can hold
     * @return a new instance of {@link IntakeSimulation} that extends over the bumper
     */
    public static IntakeSimulation OverTheBumperIntake(
            String targetedGamePieceType,
            AbstractDriveTrainSimulation driveTrainSimulation,
            Distance width,
            Distance lengthExtended,
            IntakeSide side,
            int capacity) {
        return new IntakeSimulation(
                targetedGamePieceType,
                driveTrainSimulation,
                getIntakeRectangle(driveTrainSimulation, width.in(Meters), lengthExtended.in(Meters), side),
                capacity);
    }

    private static RectangleShape getIntakeRectangle(
            AbstractDriveTrainSimulation driveTrainSimulation, double width, double lengthExtended, IntakeSide side) {
        final RectangleShape intakeRectangle = new RectangleShape(width, lengthExtended);
        intakeRectangle.rotate(
                switch (side) {
                    case LEFT, RIGHT -> 0;
                    case FRONT, BACK -> Math.toRadians(90);
                });
        final double distanceTransformed = lengthExtended / 2 - 0.01;
        intakeRectangle.translate(
                switch (side) {
                    case LEFT -> new Vec2(
                            0, driveTrainSimulation.config.bumperWidthY.in(Meters) / 2 + distanceTransformed);
                    case RIGHT -> new Vec2(
                            0, -driveTrainSimulation.config.bumperWidthY.in(Meters) / 2 - distanceTransformed);
                    case FRONT -> new Vec2(
                            driveTrainSimulation.config.bumperLengthX.in(Meters) / 2 + distanceTransformed, 0);
                    case BACK -> new Vec2(
                            -driveTrainSimulation.config.bumperLengthX.in(Meters) / 2 - distanceTransformed / 2, 0);
                });

        return intakeRectangle;
    }

    /**
     *
     *
     * <h2>Creates an Intake Simulation with a Specific Shape.</h2>
     *
     * <p>This constructor initializes an intake with a custom shape that is used when the intake is fully extended.
     *
     * @param targetedGamePieceType the type of game pieces that this intake can collect
     * @param driveTrainSimulation the chassis to which this intake is attached
     * @param shape the shape of the intake when fully extended, represented as a {@link Convex2d} object
     * @param capacity the maximum number of game pieces that the intake can hold
     */
    public IntakeSimulation(
            String targetedGamePieceType,
            AbstractDriveTrainSimulation driveTrainSimulation,
            Convex2d shape,
            int capacity) {
        super(shape);
        super.setDensity(0);

        this.targetedGamePieceType = targetedGamePieceType;
        this.gamePiecesInIntakeCount = 0;

        if (capacity > 100) throw new IllegalArgumentException("capacity too large, max is 100");
        this.capacity = capacity;

        this.gamePiecesToRemove = new ArrayDeque<>(capacity);

        this.intakeRunning = false;
        this.driveTrainSimulation = driveTrainSimulation;

        register();
    }

    /**
     *
     *
     * <h2>Turns the Intake On.</h2>
     *
     * <p>Extends the intake out from the chassis, making it part of the chassis's collision space.
     *
     * <p>Once activated, the intake is considered running and will listen for contact with
     * {@link GamePieceOnFieldSimulation} instances, allowing it to collect game pieces.
     */
    public void startIntake() {
        if (intakeRunning) return;

        driveTrainSimulation.addFixture(this);
        this.intakeRunning = true;
    }

    /**
     *
     *
     * <h2>Turns the Intake Off.</h2>
     *
     * <p>Retracts the intake into the chassis, removing it from the chassis's collision space.
     *
     * <p>Once turned off, the intake will no longer listen for or respond to contacts with
     * {@link GamePieceOnFieldSimulation} instances.
     */
    public void stopIntake() {
        if (!intakeRunning) return;

        driveTrainSimulation.removeFixture(this);
        this.intakeRunning = false;
    }

    /**
     *
     *
     * <h2>Get the amount of game pieces in the intake.</h2>
     *
     * @return the amount of game pieces stored in the intake
     */
    public int getGamePiecesAmount() {
        return gamePiecesInIntakeCount;
    }

    /**
     *
     *
     * <h2>Removes 1 game piece from the intake.</h2>
     *
     * <p>Deducts the {@link #getGamePiecesAmount()}} by 1, if there is any remaining.
     *
     * <p>This is used to obtain a game piece from the intake and move it a feeder/shooter.
     *
     * @return if there is game piece(s) remaining, and therefore retrieved
     */
    public boolean obtainGamePieceFromIntake() {
        if (gamePiecesInIntakeCount < 1) return false;
        gamePiecesInIntakeCount--;
        return true;
    }

    /**
     *
     *
     * <h2>Adds 1 game piece from the intake.</h2>
     *
     * <p>Increases the {@link #getGamePiecesAmount()}} by 1, if there is still space.
     *
     * @return if there is still space in the intake to perform this action
     */
    public boolean addGamePieceToIntake() {
        boolean toReturn = gamePiecesInIntakeCount < capacity;
        if (toReturn) gamePiecesInIntakeCount++;

        return toReturn;
    }
    /**
     *
     *
     * <h2>Adds a number of game pieces to the intake.</h2>
     *
     * If the number of pieces added would drive the intake above capacity the intake will only add pieces up to max.
     *
     * @param piecesToAdd The number of pieces to add too the intake.
     * @return Wether or not all game pieces could be added to the intake. Just because this returns false does not mean
     *     that no pieces were added.
     */
    public boolean addGamePiecesToIntake(int piecesToAdd) {
        boolean toReturn = gamePiecesInIntakeCount + piecesToAdd <= capacity;
        gamePiecesInIntakeCount = Math.min(gamePiecesInIntakeCount + piecesToAdd, capacity);
        return toReturn;
    }

    /**
     *
     *
     * <h2>Sets the amount of game pieces in the intake.</h2>
     *
     * <p>Sets the {@link #getGamePiecesAmount()}} to a given amount.
     *
     * <p>Will make sure that the amount is non-negative and does not exceed the capacity
     *
     * @return the actual (clamped) game piece count after performing this action
     */
    public int setGamePiecesCount(int gamePiecesInIntakeCount) {
        return this.gamePiecesInIntakeCount = MathUtil.clamp(gamePiecesInIntakeCount, 0, capacity);
    }

    /**
     *
     *
     * <h2>The {@link ContactListener2d} for the Intake Simulation.</h2>
     *
     * <p>This class can be added to the simulation world to detect and manage contacts between the intake and
     * {@link GamePieceOnFieldSimulation} instances of the type {@link #targetedGamePieceType}.
     *
     * <p>If contact is detected and the intake is running, the {@link GamePieceOnFieldSimulation} will be marked for
     * removal from the field.
     */
    public final class GamePieceContactListener implements ContactListener2d {
        @Override
        public void begin(CollisionData2d collision, Contact2d contact) {
            if (!intakeRunning) return;
            if (gamePiecesInIntakeCount >= capacity) return;

            final RigidBody collisionBody1 = collision.getBody1(), collisionBody2 = collision.getBody2();
            final BodyFixture2d fixture1 = collision.getFixture1(), fixture2 = collision.getFixture2();

            if (collisionBody1 instanceof GamePieceOnFieldSimulation gamePiece
                    && Objects.equals(gamePiece.type, targetedGamePieceType)
                    && fixture2 == IntakeSimulation.this) flagGamePieceForRemoval(gamePiece);
            else if (collisionBody2 instanceof GamePieceOnFieldSimulation gamePiece
                    && Objects.equals(gamePiece.type, targetedGamePieceType)
                    && fixture1 == IntakeSimulation.this) flagGamePieceForRemoval(gamePiece);

            boolean coralOrAlgaeIntake = "Coral".equals(IntakeSimulation.this.targetedGamePieceType)
                    || "Algae".equals(IntakeSimulation.this.targetedGamePieceType);
            if (collisionBody1 instanceof ReefscapeCoralAlgaeStack stack
                    && coralOrAlgaeIntake
                    && fixture2 == IntakeSimulation.this) flagGamePieceForRemoval(stack);
            else if (collisionBody2 instanceof ReefscapeCoralAlgaeStack stack
                    && coralOrAlgaeIntake
                    && fixture1 == IntakeSimulation.this) flagGamePieceForRemoval(stack);
        }

        private void flagGamePieceForRemoval(GamePieceOnFieldSimulation gamePiece) {
            if (!customIntakeCondition.test(gamePiece)) return;
            gamePiecesToRemove.add(gamePiece);
            gamePiecesInIntakeCount++;
        }

        /* functions not used */
        @Override
        public void persist(CollisionData2d collision, Contact2d oldContact, Contact2d newContact) {}

        @Override
        public void end(CollisionData2d collision, Contact2d contact) {}

        @Override
        public void destroyed(CollisionData2d collision, Contact2d contact) {}

        @Override
        public void collision(CollisionData2d collision) {}

        @Override
        public void preSolve(CollisionData2d collision, Contact2d contact) {}

        @Override
        public void postSolve(CollisionData2d collision, SolvedContact2d contact) {}
    }

    /**
     *
     *
     * <h2>Obtains a New Instance of the {@link GamePieceContactListener} for This Intake.</h2>
     *
     * @return a new {@link GamePieceContactListener} for this intake
     */
    public GamePieceContactListener getGamePieceContactListener() {
        return new GamePieceContactListener();
    }

    /**
     *
     *
     * <h2>Clears the game pieces that have been obtained by the intake.</h2>
     *
     * <p>This method is called from {@link SimulatedArena#simulationPeriodic()} to remove the
     * {@link GamePieceOnFieldSimulation} instances that have been obtained by the intake from the field.
     *
     * <p>Game pieces are marked for removal if they have come into contact with the intake during the last
     * {@link SimulatedArena#getSimulationSubTicksIn1Period()} sub-ticks. These game pieces should be removed from the
     * field to reflect their interaction with the intake.
     */
    public void removeObtainedGamePieces(SimulatedArena arena) {
        while (!gamePiecesToRemove.isEmpty()) {
            GamePieceOnFieldSimulation gamePiece = gamePiecesToRemove.poll();
            gamePiece.onIntake(this.targetedGamePieceType);
            arena.removeGamePiece(gamePiece);
        }
    }

    public void register() {
        register(SimulatedArena.getInstance());
    }

    public void register(SimulatedArena arena) {
        arena.addIntakeSimulation(this);
    }

    /**
     *
     *
     * <h2>Returns wether or not this intake is currently running</h2>
     */
    public boolean isRunning() {
        return intakeRunning;
    }

    /**
     *
     *
     * <h2>Sets a Custom Intake Condition.</h2>
     *
     * <p>This method allows the user to define a custom condition for determining whether a game piece on the field can
     * be collected by the intake. The condition is specified as a {@link Predicate} that takes a
     * {@link GamePieceOnFieldSimulation} object as input and returns a boolean value.
     *
     * <p>The condition is used to determine whether a game piece should be collected by the intake. If the predicate
     * returns true, the game piece will be collected by the intake. If the predicate returns false, the game piece will
     * not be collected by the intake. Note how this predicate is only called if the game piece is in contact with the
     * intake, and the intake is turned on, and it is the target game piece.
     *
     * @param customIntakeCondition a {@link Predicate} representing the custom condition for intake eligibility of game
     *     pieces on the field
     */
    public void setCustomIntakeCondition(Predicate<GamePieceOnFieldSimulation> customIntakeCondition) {
        this.customIntakeCondition = customIntakeCondition;
    }
}
