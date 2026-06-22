package main

import (
	"fmt"
	"math"
	"sort"
)

const (
	StepsPerMM = 100.0
	ClockSpeed = 16000000 // 16 MHz Clock

	// Distinct structural resonances
	X_ShaperFreq = 35.0 // X Axis: 35 Hz
	Y_ShaperFreq = 22.0 // Y Axis: 22 Hz

	MaxAccel   = 800.0
	StartSpeed = 4.0
)

type BinaryCommand struct {
	StepPins  uint8
	WaitTicks uint32
}

// TimeStampedStep tracks a single pin transition bound to a specific point in the future
type TimeStampedStep struct {
	TickTimestamp uint64 // Absolute CPU tick count from start of the move
	PinMask       uint8  // 0x01 for X step, 0x02 for Y step
}

func main() {
	targetX := 3.0
	targetY := 1.0
	targetSpeed := 60.0

	fmt.Printf("--- Compiling Corrected Multi-Axis Input Shaper Pipeline ---\n")

	// 1. Generate accelerated moves with absolute timestamps
	rawTimeStream, dirMask := calculateAbsoluteTimestampSteps(targetX, targetY, targetSpeed)

	// 2. Convolve X and Y independently based on their unique frequencies, then re-merge chronologically
	finalShapedStream := applyIndependentDualAxisShaper(rawTimeStream, dirMask)

	fmt.Errorf("successful compile: generated %d execution blocks for the MCU", len(finalShapedStream))
	fmt.Printf("Generated %d packed binary commands.\n", len(finalShapedStream))

	// fmt.Println(finalShapedStream)

	fmt.Println("Timing Sample Profiles (Notice the decreasing wait ticks as the motor accelerates):")
	for i := 0; i < 10; i++ {
		if i*2 < len(finalShapedStream) {
			fmt.Printf("Step Block %d -> Pulse High Wait Ticks: %d\n", i, finalShapedStream[i*2].WaitTicks)
		}
	}
}

func calculateAbsoluteTimestampSteps(targetX, targetY, targetSpeed float64) ([]TimeStampedStep, uint8) {
	var stream []TimeStampedStep

	var xDirMask, yDirMask uint8 = 0x00, 0x00
	if targetX >= 0 {
		xDirMask = 0x04
	}
	if targetY >= 0 {
		yDirMask = 0x08
	}
	dirMask := xDirMask | yDirMask

	absX := math.Abs(targetX)
	absY := math.Abs(targetY)
	totalVectorDist := math.Sqrt(absX*absX + absY*absY)
	totalVectorSteps := int(totalVectorDist * StepsPerMM)

	v0 := StartSpeed
	vMax := targetSpeed
	accel := MaxAccel

	accelDist := (vMax*vMax - v0*v0) / (2.0 * accel)
	accelSteps := int(accelDist * StepsPerMM)

	if accelSteps*2 > totalVectorSteps {
		accelSteps = totalVectorSteps / 2
		vMax = math.Sqrt(v0*v0 + 2.0*accel*(float64(accelSteps)/StepsPerMM))
	}
	decelStartStep := totalVectorSteps - accelSteps

	xCounter, yCounter := 0.0, 0.0
	xRatio := absX / totalVectorDist
	yRatio := absY / totalVectorDist

	var currentAbsoluteTicks uint64 = 0

	for s := 0; s < totalVectorSteps; s++ {
		currentVectorPos := float64(s) / StepsPerMM
		var currentVelocity float64

		if s < accelSteps {
			currentVelocity = math.Sqrt(v0*v0 + 2.0*accel*currentVectorPos)
		} else if s >= decelStartStep {
			decelDist := totalVectorDist - currentVectorPos
			currentVelocity = math.Sqrt(v0*v0 + 2.0*accel*decelDist)
		} else {
			currentVelocity = vMax
		}

		if currentVelocity < StartSpeed {
			currentVelocity = StartSpeed
		}

		stepsPerSec := currentVelocity * StepsPerMM
		secondsPerStep := 1.0 / stepsPerSec
		totalTicksForStep := uint32(secondsPerStep * float64(ClockSpeed))

		var stepMask uint8 = 0x00
		xCounter += xRatio
		yCounter += yRatio

		if xCounter >= 1.0 {
			stepMask |= 0x01 // X Step Request
			xCounter -= 1.0
		}
		if yCounter >= 1.0 {
			stepMask |= 0x02 // Y Step Request
			yCounter -= 1.0
		}

		if stepMask > 0 {
			// Increment time stream by the step window spacing
			currentAbsoluteTicks += uint64(totalTicksForStep)

			// Log the transition point
			if (stepMask & 0x01) > 0 {
				stream = append(stream, TimeStampedStep{TickTimestamp: currentAbsoluteTicks, PinMask: 0x01})
			}
			if (stepMask & 0x02) > 0 {
				stream = append(stream, TimeStampedStep{TickTimestamp: currentAbsoluteTicks, PinMask: 0x02})
			}
		}
	}
	return stream, dirMask
}

func applyIndependentDualAxisShaper(rawSteps []TimeStampedStep, dirMask uint8) []BinaryCommand {
	// Separate raw streams into isolated channels
	var xSteps, ySteps []TimeStampedStep
	for _, step := range rawSteps {
		if step.PinMask == 0x01 {
			xSteps = append(xSteps, step)
		}
		if step.PinMask == 0x02 {
			ySteps = append(ySteps, step)
		}
	}

	// Calculate specific axis time-shifts based on independent resonance loops
	xdtf64 := (1.0 / (2.0 * X_ShaperFreq)) * float64(ClockSpeed)
	ydtf64 := ((1.0 / (2.0 * Y_ShaperFreq)) * float64(ClockSpeed))
	xDelayTicks := uint64(xdtf64)
	yDelayTicks := uint64(ydtf64)

	const a1, a2 = 0.52, 0.48
	var shapedTimestamps []TimeStampedStep

	// Convolve X Channel independently
	xLimit := int(float64(len(xSteps)) * a1)
	for i, step := range xSteps {
		if i < xLimit {
			// Wavefront 1: Scaled initial speed projection
			step.TickTimestamp = uint64(float64(step.TickTimestamp) / a1)
			shapedTimestamps = append(shapedTimestamps, step)
		} else {
			// Wavefront 2: Scaled and shifted response group matching X frequency constraints
			scaledTime := uint64(float64(step.TickTimestamp) / a2)
			step.TickTimestamp = scaledTime + xDelayTicks
			shapedTimestamps = append(shapedTimestamps, step)
		}
	}

	// Convolve Y Channel independently
	yLimit := int(float64(len(ySteps)) * a1)
	for i, step := range ySteps {
		if i < yLimit {
			step.TickTimestamp = uint64(float64(step.TickTimestamp) / a1)
			shapedTimestamps = append(shapedTimestamps, step)
		} else {
			// Shift group matches Y frequency constraints completely separate from X
			scaledTime := uint64(float64(step.TickTimestamp) / a2)
			step.TickTimestamp = scaledTime + yDelayTicks
			shapedTimestamps = append(shapedTimestamps, step)
		}
	}

	// 3. Chronological Re-Ordering Sort
	sort.Slice(shapedTimestamps, func(i, j int) bool {
		return shapedTimestamps[i].TickTimestamp < shapedTimestamps[j].TickTimestamp
	})

	// 4. Group Simultaneous Steps and Transform timestamps into Relative Wait Ticks
	var commands []BinaryCommand
	if len(shapedTimestamps) == 0 {
		return commands
	}

	var lastTick uint64 = 0

	for i := 0; i < len(shapedTimestamps); i++ {
		currentStep := shapedTimestamps[i]

		// Look ahead to merge simultaneous step pulses that fall on the exact same tick
		combinedMask := currentStep.PinMask
		for i+1 < len(shapedTimestamps) && shapedTimestamps[i+1].TickTimestamp == currentStep.TickTimestamp {
			i++
			combinedMask |= shapedTimestamps[i].PinMask
		}

		deltaTicks := currentStep.TickTimestamp - lastTick
		lastTick = currentStep.TickTimestamp

		// Guard check for overflow limits on a 32-bit MCU timer register block
		if deltaTicks > math.MaxUint32 {
			commands = append(commands, BinaryCommand{StepPins: dirMask, WaitTicks: math.MaxUint32})
			deltaTicks -= math.MaxUint32
		}

		// Pulse High
		commands = append(commands, BinaryCommand{StepPins: combinedMask | dirMask, WaitTicks: uint32(deltaTicks / 2)})
		// Pulse Low
		commands = append(commands, BinaryCommand{StepPins: dirMask, WaitTicks: uint32(deltaTicks / 2)})
	}

	return commands
}
