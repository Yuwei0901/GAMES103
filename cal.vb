' NX 11.0.0.33
' Journal created by Yuwei on Thu Jul 07 19:54:15 2022 中国标准时间

'
Imports System
Imports NXOpen

Module NXJournal
Sub Main (ByVal args() As String) 

Dim theSession As NXOpen.Session = NXOpen.Session.GetSession()
' ----------------------------------------------
'   Menu: File->Open...
' ----------------------------------------------
Dim basePart1 As NXOpen.BasePart = Nothing
Dim partLoadStatus1 As NXOpen.PartLoadStatus = Nothing
basePart1 = theSession.Parts.OpenBaseDisplay("C:\Users\Yuwei\Desktop\Struct\rf_stp_fem1_sim1.sim", partLoadStatus1)

Dim workSimPart As NXOpen.CAE.SimPart = CType(theSession.Parts.BaseWork, NXOpen.CAE.SimPart)

Dim displaySimPart As NXOpen.CAE.SimPart = CType(theSession.Parts.BaseDisplay, NXOpen.CAE.SimPart)

partLoadStatus1.Dispose()
Dim markId1 As NXOpen.Session.UndoMarkId = Nothing
markId1 = theSession.SetUndoMark(NXOpen.Session.MarkVisibility.Visible, "Enter Gateway")

theSession.ApplicationSwitchImmediate("UG_APP_GATEWAY")

theSession.ApplicationSwitchImmediate("UG_APP_SFEM")

Dim simPart1 As NXOpen.CAE.SimPart = CType(workSimPart, NXOpen.CAE.SimPart)

theSession.Post.UpdateUserGroupsFromSimPart(simPart1)

Dim markId2 As NXOpen.Session.UndoMarkId = Nothing
markId2 = theSession.SetUndoMark(NXOpen.Session.MarkVisibility.Visible, "Enter Pre/Post")

' ----------------------------------------------
'   Menu: Analysis->Solve...
' ----------------------------------------------
Dim markId3 As NXOpen.Session.UndoMarkId = Nothing
markId3 = theSession.SetUndoMark(NXOpen.Session.MarkVisibility.Visible, "Start")

theSession.SetUndoMarkName(markId3, "Solve Dialog")

Dim markId4 As NXOpen.Session.UndoMarkId = Nothing
markId4 = theSession.SetUndoMark(NXOpen.Session.MarkVisibility.Invisible, "Solve")

theSession.DeleteUndoMark(markId4, Nothing)

Dim markId5 As NXOpen.Session.UndoMarkId = Nothing
markId5 = theSession.SetUndoMark(NXOpen.Session.MarkVisibility.Invisible, "Solve")

Dim theSimSolveManager As NXOpen.CAE.SimSolveManager = NXOpen.CAE.SimSolveManager.GetSimSolveManager(theSession)

Dim psolutions1(0) As NXOpen.CAE.SimSolution
Dim simSimulation1 As NXOpen.CAE.SimSimulation = CType(workSimPart.FindObject("Simulation"), NXOpen.CAE.SimSimulation)

Dim simSolution1 As NXOpen.CAE.SimSolution = CType(simSimulation1.FindObject("Solution[温度场]"), NXOpen.CAE.SimSolution)

psolutions1(0) = simSolution1
Dim numsolutionssolved1 As Integer = Nothing
Dim numsolutionsfailed1 As Integer = Nothing
Dim numsolutionsskipped1 As Integer = Nothing
theSimSolveManager.SolveChainOfSolutions(psolutions1, NXOpen.CAE.SimSolution.SolveOption.Solve, NXOpen.CAE.SimSolution.SetupCheckOption.CompleteCheckAndOutputErrors, NXOpen.CAE.SimSolution.SolveMode.Background, numsolutionssolved1, numsolutionsfailed1, numsolutionsskipped1)

theSession.DeleteUndoMark(markId5, Nothing)

theSession.SetUndoMarkName(markId3, "Solve")

' ----------------------------------------------
'   Menu: File->Save
' ----------------------------------------------
Dim simPart2 As NXOpen.CAE.SimPart = CType(workSimPart, NXOpen.CAE.SimPart)

Dim partSaveStatus1 As NXOpen.PartSaveStatus = Nothing
partSaveStatus1 = simPart2.Save(NXOpen.BasePart.SaveComponents.True, NXOpen.BasePart.CloseAfterSave.False)

partSaveStatus1.Dispose()
' ----------------------------------------------
'   Menu: Tools->Journal->Stop Recording
' ----------------------------------------------

End Sub
End Module