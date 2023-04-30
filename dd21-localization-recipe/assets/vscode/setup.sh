# open README.md at startup
VSCODE_USER_SETTINGS=/home/duckie/.local/share/code-server/User/settings.json
OUT=$(jq '. + {"workbench.startupEditor": "readme", "jupyter.widgetScriptSources": ["jsdelivr.com", "unpkg.com"], "workbench.editorAssociations": { "*.md": "vscode.markdown.preview.editor"}}' "${VSCODE_USER_SETTINGS}")
echo "${OUT}" > "${VSCODE_USER_SETTINGS}"
