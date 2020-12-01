package main

import (
	"fmt"
	"time"

	"github.com/g3n/engine/app"
	"github.com/g3n/engine/camera"
	"github.com/g3n/engine/core"
	"github.com/g3n/engine/geometry"
	"github.com/g3n/engine/gls"
	"github.com/g3n/engine/graphic"
	"github.com/g3n/engine/gui"
	"github.com/g3n/engine/light"
	"github.com/g3n/engine/material"
	"github.com/g3n/engine/math32"
	"github.com/g3n/engine/renderer"
	"github.com/g3n/engine/util/helper"
	"github.com/g3n/engine/window"
)

type Mass struct {
	M                        float32
	InitPos, Pos, Vel, Force *math32.Vector3
	Anchor                   bool
}

func NewMass(m float32, pos, vel *math32.Vector3, anchor bool) *Mass {
	return &Mass{
		M:       m,
		InitPos: pos.Clone(),
		Pos:     pos.Clone(),
		Vel:     vel.Clone(),
		Force:   math32.NewVec3(),
		Anchor:  anchor,
	}
}

func (m *Mass) Init() {
	if m.Anchor {
		m.Pos.Copy(m.InitPos)
		m.Vel.Set(0, 0, 0)
	}

	m.Force.Set(0, 0, 0)
}

func (m *Mass) Simulate(dt float32) {
	m.Vel.Add(m.Force.Clone().MultiplyScalar(dt / m.M))
	m.Pos.Add(m.Vel.Clone().MultiplyScalar(dt))
}

type Spring struct {
	Mesh                                 *graphic.Mesh
	Mass1, Mass2                         *Mass
	SpringConstantTable                  []*SpringConstant
	SpringLength, SpringFrictionConstant float32
}

type SpringConstant struct {
	Ratio, Force float32
}

func NewSpring(scene *core.Node, mass1, mass2 *Mass, springConstantTable []*SpringConstant, springLength, frictionConstant float32) *Spring {
	geom := geometry.NewCylinder(.1, float64(springLength), 12, 1, true, true)
	mat := material.NewStandard(math32.NewColor("DarkBlue"))
	mesh := graphic.NewMesh(geom, mat)
	scene.Add(mesh)

	return &Spring{
		Mesh:                   mesh,
		Mass1:                  mass1,
		Mass2:                  mass2,
		SpringConstantTable:    springConstantTable,
		SpringLength:           springLength,
		SpringFrictionConstant: frictionConstant,
	}
}

func (s *Spring) Solve() {
	springVector := s.Mass1.Pos.Clone().Sub(s.Mass2.Pos)

	r := springVector.Length()

	force := math32.NewVec3()

	if r != 0 && r > s.SpringLength {
		F := -calcSpringForce(s.SpringConstantTable, (r-s.SpringLength)/s.SpringLength)
		force.Add(springVector.Clone().MultiplyScalar(F / r))
	}

	delta := s.Mass1.Vel.Clone().Sub(s.Mass2.Vel)
	force.Sub(delta.Clone().MultiplyScalar(s.SpringFrictionConstant))

	s.Mass1.Force.Add(force)
	s.Mass2.Force.Sub(force)
}

func (s *Spring) Draw() {
	s.Mesh.SetPositionVec(s.Mass1.Pos.Clone().Add(s.Mass2.Pos).DivideScalar(2))

	delta := s.Mass2.Pos.Clone().Sub(s.Mass1.Pos).Normalize()
	quaternion := (&math32.Quaternion{}).SetFromUnitVectors(&math32.Vector3{X: 0, Y: 1, Z: 0}, delta)

	s.Mesh.SetRotationQuat(quaternion)
}

type RopeSimulation struct {
	Masses                           []*Mass
	Springs                          []*Spring
	Gravitation, AirFrictionConstant float32
}

func NewRopeSimulation(masses []*Mass, springs []*Spring, gravitation, airFrictionConstant float32) *RopeSimulation {
	return &RopeSimulation{
		Masses:              masses,
		Springs:             springs,
		Gravitation:         gravitation,
		AirFrictionConstant: airFrictionConstant,
	}
}

func (sim *RopeSimulation) Init() {
	for _, mass := range sim.Masses {
		mass.Init()
	}
}

func (sim *RopeSimulation) Solve() {
	for _, spring := range sim.Springs {
		spring.Solve()
	}

	for _, mass := range sim.Masses {
		mass.Force.Add(&math32.Vector3{
			X: 0,
			Y: -sim.Gravitation * mass.M,
			Z: 0,
		})

		mass.Force.Sub(mass.Vel.Clone().MultiplyScalar(sim.AirFrictionConstant))
	}
}

func (sim *RopeSimulation) Simulate(dt float32) {
	for _, mass := range sim.Masses {
		mass.Simulate(dt)
	}
}

func (sim *RopeSimulation) Draw() {
	for _, spring := range sim.Springs {
		spring.Draw()
	}
}

func (sim *RopeSimulation) Operate(dt float32) {
	sim.Init()
	sim.Solve()
	sim.Simulate(dt)
}

func CreateRope(scene *core.Node, numOfMasses int, mass, length float32, springConstantTable []*SpringConstant, springFrictionConstant float32, startPos, finalPos *math32.Vector3) (masses []*Mass, springs []*Spring) {
	if numOfMasses <= 1 {
		return
	}

	m := mass / float32(numOfMasses)
	springLength := length / float32(numOfMasses-1)
	delta := finalPos.Clone().Sub(startPos)
	displacement := delta.Length()
	displacementStep := displacement / float32(numOfMasses-1)
	quaternion := math32.NewQuaternion(0, 0, 0, 1).SetFromUnitVectors(math32.NewVector3(1, 0, 0), delta.Clone().Normalize())

	for i := 0; i < numOfMasses; i++ {
		masses = append(masses, NewMass(
			m,
			startPos.Clone().Add((&math32.Vector3{
				X: displacementStep * float32(i),
				Y: 0,
				Z: 0,
			}).ApplyQuaternion(quaternion)),
			math32.NewVec3(),
			false,
		))
		if i > 0 {
			springs = append(springs, NewSpring(
				scene,
				masses[i-1],
				masses[i],
				springConstantTable,
				springLength,
				springFrictionConstant,
			))
		}
	}

	return
}

func maxForce(masses []*Mass, forcePeak *float32) (maxForce float32) {
	maxForce = 0

	for _, mass := range masses {
		force := mass.Force.Length()

		if force > maxForce {
			maxForce = force
		}
	}

	if maxForce > *forcePeak {
		*forcePeak = maxForce
	}

	return
}

var rEndMass *Mass

func onKeyDown(evname string, ev interface{}) {
	ke := ev.(*window.KeyEvent)

	if ke.Key == 32 {
		rEndMass.Anchor = false
	}
}

func calcSpringForce(table []*SpringConstant, ratio float32) float32 {
	p0 := table[0]
	p1 := table[len(table)-1]
	for _, constant := range table {
		if constant.Ratio >= ratio {
			p1 = constant
			break
		}
		p0 = constant
	}

	if p0 == p1 && p1 == table[0] {
		p1 = table[1]
	}

	if p0 == p1 && p0 == table[len(table)-1] {
		p0 = table[len(table)-2]
	}

	m := (p1.Force - p0.Force) / (p1.Ratio - p0.Ratio)
	b := p0.Force

	return m*(ratio-p0.Ratio) + b
}

func calcSpringRatio(table []*SpringConstant, force float32) float32 {
	p0 := table[0]
	p1 := table[len(table)-1]
	for _, constant := range table {
		if constant.Force >= force {
			p1 = constant
			break
		}
		p0 = constant
	}

	if p0 == p1 && p1 == table[0] {
		p1 = table[1]
	}

	if p0 == p1 && p0 == table[len(table)-1] {
		p0 = table[len(table)-2]
	}

	m := (p1.Ratio - p0.Ratio) / (p1.Force - p0.Force)
	b := p0.Ratio

	return m*(force-p0.Force) + b
}

func main() {

	// Create application and scene
	a := app.App()
	scene := core.NewNode()

	// Set the scene to be managed by the gui manager
	gui.Manager().Set(scene)

	// Create perspective camera
	camTarget := &math32.Vector3{
		X: 80,
		Y: -100,
		Z: 25,
	}
	camPosition := &math32.Vector3{
		X: 200,
		Y: 100,
		Z: 100,
	}
	delta := camTarget.Clone().Sub(camPosition).Normalize()
	quaternion := (&math32.Quaternion{}).SetFromUnitVectors(&math32.Vector3{X: 0, Y: 0, Z: -1}, delta)

	cam := camera.New(1)
	cam.SetPositionVec(camPosition)
	cam.SetRotationQuat(quaternion)
	scene.Add(cam)

	// Set up orbit control for the camera
	camera.NewOrbitControl(cam)

	// Set up callback to update viewport and camera aspect ratio when the window is resized
	onResize := func(evname string, ev interface{}) {
		// Get framebuffer size and update viewport accordingly
		width, height := a.GetSize()
		a.Gls().Viewport(0, 0, int32(width), int32(height))
		// Update the camera's aspect ratio
		cam.SetAspect(float32(width) / float32(height))
	}
	a.Subscribe(window.OnWindowSize, onResize)
	onResize("", nil)

	g := float32(9.81)
	preTension := float32(2000)
	h1Segments := 100
	h1L := float32(80)
	h1M := float32(5)
	h1SpringConstantTable := []*SpringConstant{
		{Ratio: 0.0213, Force: 1000},
		{Ratio: 0.0355, Force: 2000},
		{Ratio: 0.0462, Force: 3000},
		{Ratio: 0.0533, Force: 4000},
		{Ratio: 0.0639, Force: 5000},
		{Ratio: 0.0852, Force: 6000},
		{Ratio: 0.0959, Force: 7000},
		{Ratio: 0.1030, Force: 8000},
		{Ratio: 0.1101, Force: 9000},
		{Ratio: 0.1172, Force: 10000},
		{Ratio: 0.1243, Force: 11000},
		{Ratio: 0.1278, Force: 12000},
	}
	h2Segments := 100
	h2L := float32(100)
	h2M := float32(5)
	h2SpringConstantTable := h1SpringConstantTable
	rSegments := 100
	rL := float32(100)
	rM := float32(5)
	rSpringConstantTable := []*SpringConstant{
		{Ratio: 0, Force: 0},
		{Ratio: 1, Force: 25000},
	}

	// Create a rope and add it to the scene
	h1Masses, h1Springs := CreateRope(
		scene,
		h1Segments,
		h1M,
		h1L*(1-calcSpringRatio(h1SpringConstantTable, preTension)),
		h1SpringConstantTable,
		0.2,
		&math32.Vector3{
			X: 0,
			Y: 0,
			Z: 0,
		},
		&math32.Vector3{
			X: h1L,
			Y: 0,
			Z: 0,
		},
	)
	h1Masses[0].Anchor = true

	h2Masses, h2Springs := CreateRope(
		scene,
		h2Segments,
		h2M,
		h2L*(1-calcSpringRatio(h2SpringConstantTable, preTension)),
		h2SpringConstantTable,
		0.2,
		&math32.Vector3{
			X: h1L,
			Y: 0,
			Z: 0,
		},
		&math32.Vector3{
			X: h1L + h2L,
			Y: 0,
			Z: 0,
		},
	)
	h2Masses[len(h2Masses)-1].Anchor = true

	rMasses, rSprings := CreateRope(
		scene,
		rSegments,
		rM,
		rL,
		rSpringConstantTable,
		0.2,
		&math32.Vector3{
			X: h1L,
			Y: 0,
			Z: 0,
		},
		&math32.Vector3{
			X: h1L,
			Y: 0,
			Z: 50,
		},
	)
	rStartMass := rMasses[0]
	rEndMass = rMasses[len(rMasses)-1]
	rEndMass.M = 100.0
	/*t := math.Sqrt(float32(2 * rL / g))
	v := g * float32(t)
	rEndMass.Vel = &math32.Vector3{
		X: 0,
		Y: float32(-v),
		Z: 0,
	}*/
	rEndMass.Anchor = true

	masses := []*Mass{}
	masses = append(masses, h1Masses...)
	masses = append(masses, h2Masses...)
	masses = append(masses, rMasses...)

	springs := []*Spring{}
	springs = append(springs, h1Springs...)
	springs = append(springs, h2Springs...)
	springs = append(springs, rSprings...)
	springs = append(springs, NewSpring(
		scene,
		h1Masses[len(h1Masses)-1],
		rMasses[0],
		h1SpringConstantTable,
		0.05,
		0.2,
	))
	springs = append(springs, NewSpring(
		scene,
		h2Masses[0],
		rMasses[0],
		h2SpringConstantTable,
		0.05,
		0.2,
	))

	ropeSimulation := NewRopeSimulation(
		masses,
		springs,
		g,
		0.02,
	)

	// Create and add lights to the scene
	scene.Add(
		light.NewAmbient(
			&math32.Color{
				R: 1.0,
				G: 1.0,
				B: 1.0,
			},
			0.8,
		),
	)
	pointLight := light.NewPoint(
		&math32.Color{
			R: 1,
			G: 1,
			B: 1,
		},
		5.0,
	)
	pointLight.SetPosition(1, 0, 2)
	scene.Add(pointLight)

	// Create and add an axis helper to the scene
	scene.Add(helper.NewAxes(0.5))

	// Set background color to gray
	a.Gls().ClearColor(0.5, 0.5, 0.5, 1.0)

	h1ForcePeak := float32(0)
	h2ForcePeak := float32(0)
	rForcePeak := float32(0)
	maxJumperY := float32(0)
	maxHighlineY := float32(0)

	a.Subscribe(window.OnKeyDown, onKeyDown)

	// Run the application
	a.Run(func(renderer *renderer.Renderer, deltaTime time.Duration) {
		OpsPerFrame := 10000
		dt := float32(deltaTime.Nanoseconds()) * 1e-9 / float32(OpsPerFrame)

		for i := 0; i < OpsPerFrame; i++ {
			ropeSimulation.Operate(dt)
		}

		h1Force := maxForce(h1Masses, &h1ForcePeak)
		h2Force := maxForce(h2Masses, &h2ForcePeak)
		rForce := maxForce(rMasses, &rForcePeak)

		if rEndMass.Pos.Y < maxJumperY {
			maxJumperY = rEndMass.Pos.Y
		}

		if rStartMass.Pos.Y < maxHighlineY {
			maxHighlineY = rStartMass.Pos.Y
		}

		if rEndMass.Anchor == true {
			h1ForcePeak = 0
			h2ForcePeak = 0
			rForcePeak = 0
			maxJumperY = 0
			maxHighlineY = 0
		}

		fmt.Print("\033[H\033[2J")
		fmt.Printf("dt: %vs\n", dt)
		fmt.Printf("h1 force: %vN\n", h1Force)
		fmt.Printf("h1 force peak: %vN\n", h1ForcePeak)
		fmt.Printf("h2 force: %vN\n", h2Force)
		fmt.Printf("h2 force peak: %vN\n", h2ForcePeak)
		fmt.Printf("r force: %vN\n", rForce)
		fmt.Printf("r force peak: %vN\n", rForcePeak)
		fmt.Printf("highline pos: %vm\n", rStartMass.Pos)
		fmt.Printf("highline max y: %vm\n", maxHighlineY)
		fmt.Printf("jumper pos: %vm\n", rEndMass.Pos)
		fmt.Printf("jumper max y: %vm\n", maxJumperY)

		ropeSimulation.Draw()
		a.Gls().Clear(gls.DEPTH_BUFFER_BIT | gls.STENCIL_BUFFER_BIT | gls.COLOR_BUFFER_BIT)
		renderer.Render(scene, cam)
	})
}
