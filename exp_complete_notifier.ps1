# call syntax: title, subtext, excel file path
$Title = New-BTText -Text $args[0]
$Subtext = New-BTText -Text $args[1]
$FileName = $args[2]

# get the full path to the excel file
$FilePath = Join-Path $PSScriptRoot $FileName

# set sound
$Audio = New-BTAudio -Source ms-winsoundevent:Notification.Looping.Alarm5

# set buttons
$Action = New-BTAction -SnoozeAndDismiss


# create notification
$Binding = New-BTBinding -Children $Title, $Subtext 
$Visual = New-BTVisual -BindingGeneric $Binding
$Content = New-BTContent -Visual $Visual -Actions $Action -Audio $Audio -Scenario Alarm -Launch $FilePath -ActivationType Protocol

# send notification
Submit-BTNotification -Content $Content